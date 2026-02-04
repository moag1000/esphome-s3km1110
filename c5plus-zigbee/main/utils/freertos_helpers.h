/**
 * @file freertos_helpers.h
 * @brief FreeRTOS utility macros and helpers
 *
 * Provides:
 * - Scoped mutex guard macros for safer critical section management
 * - PSRAM task creation helpers for memory optimization
 *
 * @note Do NOT use return, break, continue, or goto inside MUTEX_GUARD blocks
 *       as this will skip the automatic mutex release.
 */

#ifndef FREERTOS_HELPERS_H
#define FREERTOS_HELPERS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * PSRAM Task Creation Helpers
 * ============================================================================
 * These helpers allow creating FreeRTOS tasks with stacks allocated in PSRAM
 * (external SPI RAM), freeing up precious internal RAM for DMA and fast access.
 *
 * Use for non-latency-critical tasks like:
 * - System monitoring
 * - API servers
 * - Background processing
 *
 * Do NOT use for:
 * - Zigbee stack tasks (requires fast internal RAM)
 * - WiFi tasks (DMA requirements)
 * - Interrupt-heavy tasks
 */

/**
 * @brief Handle for tasks created with PSRAM stack
 *
 * This structure tracks the resources allocated for a PSRAM-backed task.
 * Must be preserved for the lifetime of the task to enable proper cleanup.
 */
typedef struct {
    TaskHandle_t task_handle;   /**< FreeRTOS task handle */
    StaticTask_t *tcb;          /**< Task Control Block (in PSRAM) */
    StackType_t *stack;         /**< Task stack buffer (in PSRAM) */
    uint32_t stack_size;        /**< Stack size in bytes */
} psram_task_handle_t;

/**
 * @brief Create a task with stack allocated in PSRAM
 *
 * Creates a FreeRTOS task using xTaskCreateStatic with both the Task Control
 * Block (TCB) and stack buffer allocated in PSRAM. This frees internal RAM
 * for tasks that don't require low-latency memory access.
 *
 * @param task_func     Task function pointer
 * @param name          Task name (for debugging)
 * @param stack_size    Stack size in bytes (will be converted to words)
 * @param param         Parameter passed to task function
 * @param priority      Task priority
 * @param out_handle    Pointer to receive the PSRAM task handle (must not be NULL)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if PSRAM allocation fails
 * @return ESP_ERR_INVALID_ARG if out_handle is NULL
 *
 * @note The out_handle must be preserved for the lifetime of the task.
 *       Call psram_task_delete() to clean up resources.
 *
 * @code
 * static psram_task_handle_t s_monitor_task;
 *
 * esp_err_t start_monitor(void) {
 *     return psram_task_create(monitor_func, "monitor", 4096, NULL, 5, &s_monitor_task);
 * }
 *
 * void stop_monitor(void) {
 *     psram_task_delete(&s_monitor_task);
 * }
 * @endcode
 */
static inline esp_err_t psram_task_create(
    TaskFunction_t task_func,
    const char *name,
    uint32_t stack_size,
    void *param,
    UBaseType_t priority,
    psram_task_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize handle */
    out_handle->task_handle = NULL;
    out_handle->tcb = NULL;
    out_handle->stack = NULL;
    out_handle->stack_size = stack_size;

    /*
     * TCB (Task Control Block) MUST be in internal RAM on ESP32-C5.
     * FreeRTOS validates TCB memory location with xPortCheckValidTCBMem().
     * Only the stack can be in PSRAM.
     */
    out_handle->tcb = heap_caps_malloc(sizeof(StaticTask_t),
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (out_handle->tcb == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Allocate stack in PSRAM (fallback to internal if PSRAM unavailable) */
    out_handle->stack = heap_caps_malloc(stack_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (out_handle->stack == NULL) {
        /* Fallback to internal RAM */
        out_handle->stack = heap_caps_malloc(stack_size, MALLOC_CAP_INTERNAL);
        if (out_handle->stack == NULL) {
            heap_caps_free(out_handle->tcb);
            out_handle->tcb = NULL;
            return ESP_ERR_NO_MEM;
        }
    }

    /* Create task with static allocation */
    out_handle->task_handle = xTaskCreateStatic(
        task_func,
        name,
        stack_size / sizeof(StackType_t),  /* Convert bytes to words */
        param,
        priority,
        out_handle->stack,
        out_handle->tcb
    );

    if (out_handle->task_handle == NULL) {
        heap_caps_free(out_handle->stack);
        heap_caps_free(out_handle->tcb);
        out_handle->stack = NULL;
        out_handle->tcb = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Delete a PSRAM-backed task and free its resources
 *
 * Deletes the task and frees the PSRAM-allocated stack and TCB.
 *
 * @warning This function must NOT be called from the task being deleted.
 *          Tasks should signal another task to perform the cleanup.
 *
 * @warning After calling this function, the task_handle becomes invalid.
 *          Do not access task_handle->task_handle after deletion.
 *
 * @param task_handle   Pointer to the PSRAM task handle
 *
 * @note If the task has already been deleted (task_handle is NULL),
 *       this function will only free the memory resources.
 */
static inline void psram_task_delete(psram_task_handle_t *task_handle)
{
    if (task_handle == NULL) {
        return;
    }

    /* Delete the task if it exists */
    if (task_handle->task_handle != NULL) {
        vTaskDelete(task_handle->task_handle);
        task_handle->task_handle = NULL;
    } else if (task_handle->stack != NULL) {
        /*
         * Task was marked as self-deleted via psram_task_mark_deleted().
         * The task called mark_deleted just before vTaskDelete(NULL).
         * Yield to ensure the task has fully exited before freeing its stack.
         */
        taskYIELD();
    }

    /* Free PSRAM resources */
    if (task_handle->stack != NULL) {
        heap_caps_free(task_handle->stack);
        task_handle->stack = NULL;
    }

    if (task_handle->tcb != NULL) {
        heap_caps_free(task_handle->tcb);
        task_handle->tcb = NULL;
    }

    task_handle->stack_size = 0;
}

/**
 * @brief Mark a PSRAM task as self-deleted
 *
 * Call this from within a task that deletes itself (vTaskDelete(NULL)).
 * This clears the task handle so that psram_task_delete() knows the task
 * has already been deleted and only needs to free memory resources.
 *
 * @param task_handle   Pointer to the PSRAM task handle
 *
 * @note After calling this, call vTaskDelete(NULL) to delete the task.
 *       Another task must later call psram_task_delete() to free memory.
 *
 * @code
 * static psram_task_handle_t s_my_task;
 *
 * void my_task_func(void *param) {
 *     while (running) {
 *         // ... task work ...
 *     }
 *     // Task is exiting - mark as self-deleted
 *     psram_task_mark_deleted(&s_my_task);
 *     vTaskDelete(NULL);
 * }
 * @endcode
 */
static inline void psram_task_mark_deleted(psram_task_handle_t *task_handle)
{
    if (task_handle != NULL) {
        task_handle->task_handle = NULL;
    }
}

/**
 * @brief Check if a PSRAM task is valid and running
 *
 * @param task_handle   Pointer to the PSRAM task handle
 * @return true if task is created and not deleted
 */
static inline bool psram_task_is_valid(const psram_task_handle_t *task_handle)
{
    return (task_handle != NULL && task_handle->task_handle != NULL);
}

/**
 * @brief Get the FreeRTOS task handle from a PSRAM task handle
 *
 * @param task_handle   Pointer to the PSRAM task handle
 * @return TaskHandle_t or NULL if invalid
 */
static inline TaskHandle_t psram_task_get_handle(const psram_task_handle_t *task_handle)
{
    return (task_handle != NULL) ? task_handle->task_handle : NULL;
}

/**
 * @brief Scoped mutex guard macro
 *
 * Acquires the mutex at block entry and automatically releases it at block exit.
 * Uses portMAX_DELAY for acquisition, blocking indefinitely until available.
 *
 * Usage:
 * @code
 *   MUTEX_GUARD(s_data_mutex) {
 *       // Critical section code
 *       s_shared_counter++;
 *       process_shared_data();
 *   }
 *   // Mutex automatically released here
 * @endcode
 *
 * @warning Do NOT use return, break, continue, or goto inside the block.
 *          These will bypass the automatic mutex release and cause deadlocks.
 *
 * @param mutex The mutex handle (SemaphoreHandle_t)
 */
#define MUTEX_GUARD(mutex)                                                 \
    for (int _guard = (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE ? 1 : 0); \
         _guard;                                                           \
         _guard = 0, xSemaphoreGive(mutex))

/**
 * @brief Scoped mutex guard with timeout
 *
 * Attempts to acquire the mutex with a specified timeout. The acquired variable
 * indicates whether acquisition succeeded, allowing the caller to handle timeout.
 *
 * Usage:
 * @code
 *   bool acquired;
 *   MUTEX_GUARD_TIMEOUT(s_data_mutex, pdMS_TO_TICKS(100), acquired) {
 *       if (acquired) {
 *           // Critical section code
 *           s_shared_counter++;
 *       } else {
 *           ESP_LOGW(TAG, "Failed to acquire mutex within timeout");
 *       }
 *   }
 *   // Mutex automatically released if it was acquired
 * @endcode
 *
 * @warning Do NOT use return, break, continue, or goto inside the block.
 *          These will bypass the automatic mutex release and cause deadlocks.
 *
 * @param mutex The mutex handle (SemaphoreHandle_t)
 * @param timeout_ticks Timeout in FreeRTOS ticks (use pdMS_TO_TICKS for ms)
 * @param acquired Variable to receive acquisition result (bool)
 */
#define MUTEX_GUARD_TIMEOUT(mutex, timeout_ticks, acquired)                     \
    for (int _guard = ((acquired = (xSemaphoreTake(mutex, timeout_ticks) == pdTRUE)), 1); \
         _guard;                                                                \
         _guard = 0, (acquired ? xSemaphoreGive(mutex) : (void)0))

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_HELPERS_H */
