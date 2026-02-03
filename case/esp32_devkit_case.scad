////////////////////////////////////////////////////////////////////////////////
// ESP32 DevKit Parametric Case with Snap-Fit, Compliant Buttons & LED Windows
//
// Author: Generated for Waveshare ESP32-C5-WIFI6-KIT / ESP32-S3-DevKitC-1
// License: CC BY-SA 4.0
//
// Usage:
//   1. Measure your board and fill in the parameters below
//   2. Render bottom: set RENDER_PART = "bottom"
//   3. Render lid:    set RENDER_PART = "lid"
//   4. Export STL and slice
//
////////////////////////////////////////////////////////////////////////////////

/* [Render Control] */
// Which part to render
RENDER_PART = "both"; // ["bottom", "lid", "both", "exploded"]
// Exploded view offset (only for "exploded")
explode_offset = 30;

/* [Board Dimensions] */
// Board length (X direction, along USB)
board_len = 52.0;
// Board width (Y direction)
board_wid = 28.0;
// Board thickness (PCB only)
board_thick = 1.6;
// Max component height on TOP of board
board_top_height = 8.0;
// Max component height on BOTTOM (pins, solder joints)
board_bottom_height = 3.0;

/* [Case Parameters] */
// Wall thickness
wall = 2.0;
// Clearance around board (X/Y)
clearance_xy = 0.5;
// Clearance above board components
clearance_z = 1.0;
// Corner radius
corner_r = 3.0;
// Board support ledge width
ledge_width = 1.5;
// Board support ledge height from bottom interior
ledge_height = 3.5;

/* [Snap-Fit Parameters] */
// Snap hook thickness (flexible arm)
snap_thickness = 1.4;
// Snap hook length
snap_length = 6.0;
// Snap hook catch height
snap_catch = 1.0;
// Snap hook catch angle
snap_angle = 45;
// Number of snaps per long side
snaps_per_side = 2;
// Snap clearance (printing tolerance)
snap_clearance = 0.15;

/* [USB Cutout] */
// USB port width
usb_width = 12.0;
// USB port height
usb_height = 7.0;
// USB port center offset from board bottom edge (Y)
usb_y_offset = 0; // 0 = centered
// USB port height offset from board bottom
usb_z_offset = 1.5;
// Enable USB cutout
usb_enable = true;

/* [Button Positions - relative to board corner (USB side, left)] */
// Reset/EN button [x, y] from reference corner
btn_reset_pos = [3.5, 14.0];
// Reset button diameter
btn_reset_dia = 4.0;
// Boot/IO0 button [x, y] from reference corner
btn_boot_pos = [3.5, 7.0];
// Boot button diameter
btn_boot_dia = 4.0;
// Button height above board surface
btn_height_above_board = 2.5;
// Enable buttons in lid
btn_enable = true;

/* [Compliant Button Mechanism] */
// Flex arm length
flex_length = 8.0;
// Flex arm width
flex_width = 4.0;
// Flex arm thickness (critical for flexibility - 0.8-1.2mm for PETG)
flex_thickness = 1.0;
// Pusher pin diameter
pusher_dia = 2.5;
// Pusher pin extra length (into button)
pusher_extra = 0.3;
// Button travel distance
btn_travel = 0.8;
// Finger pad diameter (external)
finger_pad_dia = 8.0;
// Finger pad height (external)
finger_pad_height = 1.5;

/* [LED Positions - relative to board corner (USB side, left)] */
// RGB LED position [x, y]
led_rgb_pos = [45.0, 14.0];
// RGB LED window diameter
led_rgb_dia = 4.0;
// CHG LED position [x, y]
led_chg_pos = [40.0, 5.0];
// CHG LED window diameter
led_chg_dia = 3.0;
// PWR LED position [x, y]
led_pwr_pos = [40.0, 23.0];
// PWR LED window diameter
led_pwr_dia = 3.0;
// LED height above board
led_height_above_board = 1.5;
// Enable LED windows
led_enable = true;
// LED window style: 0=hole, 1=thin membrane (0.4mm), 2=light pipe stub
led_style = 1;
// Membrane thickness (for style 1)
led_membrane = 0.4;

/* [Ventilation] */
// Enable ventilation slots
vent_enable = true;
// Vent slot width
vent_width = 2.0;
// Vent slot length
vent_length = 15.0;
// Number of vent slots per side
vent_count = 3;
// Vent spacing
vent_spacing = 4.0;

/* [Mounting] */
// Enable mounting tabs
mount_enable = false;
// Mounting hole diameter
mount_hole_dia = 3.2;
// Mounting tab width
mount_tab_width = 10.0;
// Mounting tab length (extension from case)
mount_tab_length = 8.0;

////////////////////////////////////////////////////////////////////////////////
// Calculated values
////////////////////////////////////////////////////////////////////////////////

// Interior dimensions
int_len = board_len + 2 * clearance_xy;
int_wid = board_wid + 2 * clearance_xy;
int_height_bottom = ledge_height + board_thick + board_top_height + clearance_z;
int_height_lid = wall + snap_catch;

// Exterior dimensions
ext_len = int_len + 2 * wall;
ext_wid = int_wid + 2 * wall;
ext_height_bottom = int_height_bottom + wall;
ext_height_lid = int_height_lid + wall;

// Board position inside case (XY from interior corner)
board_x = clearance_xy;
board_y = clearance_xy;
board_z = ledge_height; // Z from interior bottom

// Snap positions
snap_positions_x = [ext_len * 0.25, ext_len * 0.75];
snap_positions_y = [ext_wid * 0.25, ext_wid * 0.75];

////////////////////////////////////////////////////////////////////////////////
// Modules
////////////////////////////////////////////////////////////////////////////////

// Rounded box primitive
module rounded_box(size, r, center=false) {
    hull() {
        for (x = [r, size[0]-r])
            for (y = [r, size[1]-r])
                translate([x, y, 0])
                    cylinder(h=size[2], r=r, $fn=32);
    }
    if (center) translate([-size[0]/2, -size[1]/2, -size[2]/2]) children();
}

// Bottom case
module case_bottom() {
    difference() {
        union() {
            // Main shell
            rounded_box([ext_len, ext_wid, ext_height_bottom], corner_r);

            // Mounting tabs
            if (mount_enable) {
                for (x = [ext_len/2])
                    for (y = [-mount_tab_length, ext_wid]) {
                        translate([x - mount_tab_width/2, y < 0 ? y : y, 0])
                            cube([mount_tab_width, mount_tab_length, wall]);
                    }
            }
        }

        // Interior cavity
        translate([wall, wall, wall])
            rounded_box([int_len, int_wid, int_height_bottom + 1], corner_r - wall/2);

        // USB cutout
        if (usb_enable) {
            usb_y = wall + int_wid/2 + usb_y_offset - usb_width/2;
            usb_z = wall + ledge_height + usb_z_offset;
            translate([-1, usb_y, usb_z])
                cube([wall + 2, usb_width, usb_height]);
        }

        // Ventilation slots (sides)
        if (vent_enable) {
            vent_start = (ext_len - (vent_count - 1) * vent_spacing - vent_length) / 2;
            for (i = [0:vent_count-1]) {
                // Left side
                translate([vent_start + vent_length/2, -1, ext_height_bottom/2])
                    rotate([-90, 0, 0])
                        rounded_slot(vent_length, vent_width, wall + 2);
                // Right side
                translate([vent_start + vent_length/2, ext_wid - wall - 1, ext_height_bottom/2])
                    rotate([-90, 0, 0])
                        rounded_slot(vent_length, vent_width, wall + 2);
            }
        }

        // Mounting holes
        if (mount_enable) {
            for (x = [ext_len/2])
                for (y = [-mount_tab_length/2, ext_wid + mount_tab_length/2]) {
                    translate([x, y, -1])
                        cylinder(d=mount_hole_dia, h=wall + 2, $fn=24);
                }
        }
    }

    // Board support ledges (4 corners)
    ledge_inset = 2;
    for (x = [wall + ledge_inset, ext_len - wall - ledge_inset - ledge_width])
        for (y = [wall + ledge_inset, ext_wid - wall - ledge_inset - ledge_width]) {
            translate([x, y, wall])
                cube([ledge_width, ledge_width, ledge_height]);
        }

    // Snap-fit receptacles (cutouts in top edge)
    // These are created as part of the lid engagement
}

// Rounded slot for vents
module rounded_slot(length, width, depth) {
    hull() {
        for (x = [-length/2 + width/2, length/2 - width/2])
            translate([x, 0, 0])
                cylinder(d=width, h=depth, $fn=16);
    }
}

// Snap-fit hook (for lid)
module snap_hook() {
    // Flexible arm
    translate([0, 0, 0])
        cube([snap_thickness, snap_length, snap_catch + wall]);
    // Catch
    translate([0, snap_length - snap_thickness, 0])
        rotate([snap_angle, 0, 0])
            cube([snap_thickness, snap_catch * 1.5, snap_catch]);
}

// Compliant button mechanism
module compliant_button(btn_pos, btn_dia, label="") {
    // Position relative to case interior
    bx = wall + board_x + btn_pos[0];
    by = wall + board_y + btn_pos[1];

    // Calculate pusher length
    lid_bottom_z = ext_height_bottom;
    board_top_z = wall + ledge_height + board_thick;
    btn_top_z = board_top_z + btn_height_above_board;
    pusher_len = lid_bottom_z - btn_top_z + pusher_extra + btn_travel;

    difference() {
        union() {
            // Finger pad (raised button on exterior)
            translate([bx, by, ext_height_lid - finger_pad_height])
                cylinder(d=finger_pad_dia, h=finger_pad_height, $fn=32);
        }

        // Clearance slot for flex arm movement
        translate([bx - flex_width/2 - 0.5, by - flex_length - 1, -0.1])
            cube([flex_width + 1, flex_length + 1 + btn_dia/2, wall + 0.2]);
    }

    // Flex arm with pusher
    translate([bx, by, 0]) {
        difference() {
            union() {
                // Flex arm (anchored at +Y, free end at button)
                translate([-flex_width/2, 0, 0])
                    cube([flex_width, flex_length, flex_thickness]);

                // Pusher pin
                translate([0, 0, -pusher_len + flex_thickness])
                    cylinder(d=pusher_dia, h=pusher_len, $fn=24);

                // Reinforcement at anchor
                translate([-flex_width/2 - 0.5, flex_length - 1, 0])
                    cube([flex_width + 1, 1.5, flex_thickness + 1]);
            }

            // Stress relief notches at anchor
            for (x = [-flex_width/2 - 0.1, flex_width/2 - 0.3])
                translate([x, flex_length - 0.5, -0.1])
                    cube([0.4, 1, flex_thickness + 0.2]);
        }
    }
}

// LED window
module led_window(led_pos, led_dia) {
    lx = wall + board_x + led_pos[0];
    ly = wall + board_y + led_pos[1];

    translate([lx, ly, 0]) {
        if (led_style == 0) {
            // Simple hole
            translate([0, 0, -0.1])
                cylinder(d=led_dia, h=ext_height_lid + 0.2, $fn=24);
        } else if (led_style == 1) {
            // Thin membrane
            translate([0, 0, -0.1])
                cylinder(d=led_dia, h=ext_height_lid - led_membrane + 0.1, $fn=24);
            // Countersink on top
            translate([0, 0, ext_height_lid - 0.5])
                cylinder(d1=led_dia, d2=led_dia + 1, h=0.6, $fn=24);
        } else {
            // Light pipe stub (hole with raised ring)
            translate([0, 0, -0.1])
                cylinder(d=led_dia - 1, h=ext_height_lid + 0.2, $fn=24);
        }
    }
}

// Lid
module case_lid() {
    difference() {
        union() {
            // Main lid plate
            rounded_box([ext_len, ext_wid, ext_height_lid], corner_r);

            // Inner lip (fits inside bottom case)
            lip_clearance = snap_clearance;
            translate([wall + lip_clearance, wall + lip_clearance, -wall])
                difference() {
                    rounded_box([int_len - 2*lip_clearance, int_wid - 2*lip_clearance, wall + 0.1],
                               max(0.5, corner_r - wall - lip_clearance));
                    // Hollow out the lip
                    translate([wall/2, wall/2, -0.1])
                        rounded_box([int_len - 2*lip_clearance - wall,
                                    int_wid - 2*lip_clearance - wall,
                                    wall + 0.3],
                                   max(0.5, corner_r - wall*1.5 - lip_clearance));
                }

            // Snap hooks on lip
            for (x = snap_positions_x) {
                // Front
                translate([x - snap_thickness/2, wall + snap_clearance, -wall])
                    rotate([0, 0, 180])
                        rotate([0, 0, 90])
                            snap_hook();
                // Back
                translate([x + snap_thickness/2, ext_wid - wall - snap_clearance, -wall])
                    rotate([0, 0, 90])
                        snap_hook();
            }
            for (y = snap_positions_y) {
                // Left
                translate([wall + snap_clearance, y + snap_thickness/2, -wall])
                    rotate([0, 0, 180])
                        snap_hook();
                // Right
                translate([ext_len - wall - snap_clearance, y - snap_thickness/2, -wall])
                    snap_hook();
            }

            // Compliant buttons
            if (btn_enable) {
                translate([0, 0, 0]) {
                    compliant_button(btn_reset_pos, btn_reset_dia, "RST");
                    compliant_button(btn_boot_pos, btn_boot_dia, "BOOT");
                }
            }
        }

        // LED windows
        if (led_enable) {
            led_window(led_rgb_pos, led_rgb_dia);
            led_window(led_chg_pos, led_chg_dia);
            led_window(led_pwr_pos, led_pwr_dia);
        }

        // Button clearance cutouts (for flex arm travel)
        if (btn_enable) {
            // Already handled in compliant_button module
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Render
////////////////////////////////////////////////////////////////////////////////

if (RENDER_PART == "bottom") {
    case_bottom();
} else if (RENDER_PART == "lid") {
    // Flip lid for printing
    translate([0, ext_wid, ext_height_lid])
        rotate([180, 0, 0])
            case_lid();
} else if (RENDER_PART == "both") {
    case_bottom();
    translate([0, 0, ext_height_bottom])
        case_lid();
} else if (RENDER_PART == "exploded") {
    case_bottom();
    translate([0, 0, ext_height_bottom + explode_offset])
        case_lid();
}

////////////////////////////////////////////////////////////////////////////////
// Info output
////////////////////////////////////////////////////////////////////////////////

echo("=== Case Dimensions ===");
echo(str("Exterior: ", ext_len, " x ", ext_wid, " x ", ext_height_bottom + ext_height_lid, " mm"));
echo(str("Interior: ", int_len, " x ", int_wid, " x ", int_height_bottom, " mm"));
echo(str("Board position from interior corner: X=", board_x, " Y=", board_y, " Z=", board_z));
