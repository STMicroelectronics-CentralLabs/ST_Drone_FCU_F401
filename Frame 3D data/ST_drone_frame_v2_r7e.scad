/* [Quadcopter Size] */

//The distance between the centres of right and left front motors
Width = 138.5 / 1.41;// 145 / 1.41

//The distance between the centres of front and back left motors
Length = Width;

//Flight Controller Width
FC_Width = 40;

//Flight Controller Length
FC_Length = 40;

//Joint bolt diameter
bolt_r = 2.1 / 2;

//The thickness of your quadcopter frame determines how strong it is, how flexible it is, how much it dampens vibration, etc.
Frame_Thickness = 1.5;

//Width of Frame Surfaces at narrowest point
Surface_Width = 1.9;

//base_Thickness= Frame_Thickness+.5;

/* [Motor Holders] */
//Actual motor diameter
Motor_Diameter = 8.2;// [6,7,8.5]

//Motor hole scaling.  Use this if your holes are the wrong size.  The frame is built with scaled_motor_diameter=Motor_Hole_Scaling*Motor_Diameter, so 1.0 is the no-change default.
Motor_Hole_Scaling = 1.0;

//Total motor cap height (extends above the frame)
Motor_Cap = 17;

//The thickness of the motor cap wall
Motor_Cap_Wall_Thickness = 1.45;

//The size of the side opening on the motor caps
Motor_Cap_Side_Opening =  2.5;

//This adds a slight bevel inside the motor cap so that you don't have to cut away any material to insert motors.
Motor_Cap_Inner_Bevel = 0;

//Actual rotor diameter
Rotor_Diameter = 69.5;//70+3.5;
Rotor_Diameter_h = 8;

// Roter Angle
roter_angle = -2.5;

//Total rotor guard height
Rotor_Gud_h = 7.5; //*1.5;

//The thickness of the rotor guard wall
Rotor_Gud_Wall_Thickness = .95;//1.0

//The thickness of your quadcopter frame determines how strong it is, how flexible it is, how much it dampens vibration, etc.
Rotor_Gud_Frame_Thickness = 1.65;

// Top-Buttom flame joint hight
Frame_Base_Hight = 10 + Frame_Thickness + 1.6;
//AB: Frame_Base_Hight = 15 + Frame_Thickness + 1.6;
Frame_Spacer_Hight = 10;
//Frame_Joint_Hight= Frame_Base_Hight + Frame_Spacer_Hight;

//Which parts do you want to show
Which_Parts= "print"; // ["frame":Whole_frame, "print":print_mode]
Guarde_mode= "mesh"; // ["mesh" or "nomesh"]

Resolution= 100; //[20,50,100]

//calculated variables
$fn=Resolution;
FC_Band=Surface_Width;

scaled_motor_diameter=Motor_Hole_Scaling*Motor_Diameter;
cap_wall_thickness=Motor_Cap_Wall_Thickness;
cap_wall_opening=Motor_Cap_Side_Opening;

Rotor_Frame_Thickness = 2.0;
Rotor_Frame_Num = 3;
Rotor_Frame_Angle = 15;
Rotor_Frame_r = 10;
Rotor_Frame_h_offset = -2.5;

Landing_leg_h = 7.5;

// main 
union()
{
    
    // External Motor and Propeller Guards 
    for(r=[0:180:360])
    {
        rotate([0,0,r])
        translate([Width/2,Length/2,-(Motor_Diameter/2+Motor_Cap_Wall_Thickness)*tan(roter_angle)+0.25]) //-.1])
        rotate([0,0,135])
        rotate([roter_angle,0,0])
        {
            roter_cw();
            %translate([0,0,Rotor_Diameter_h/2+3])cylinder(r=Rotor_Diameter/2,h=Rotor_Diameter_h, center=true);
        }
    }
    
    for(r=[90:180:270])
        {
        rotate([0,0,r])
        translate([Length/2,Width/2,-(Motor_Diameter/2+Motor_Cap_Wall_Thickness)*tan(roter_angle)+0.25])//AB:-.1])
        rotate([0,0,135])
        rotate([roter_angle,0,0])
        {
            roter_ccw();
            %translate([0,0,Rotor_Diameter_h/2+3])cylinder(r=Rotor_Diameter/2,h=Rotor_Diameter_h, center=true);
        }
    }
    
    
    difference()
    {
        union()
        {
            // Upper Part of Center
            guard_centre();
            // Below Part of Center
            //FCU_support();
            
            for(i=[0:360/2:360])
            {
                rotate(i)strut_up(x=Width/2,y=Length/2,apex=FC_Length/2-FC_Band,band=FC_Band,ir=Rotor_Diameter/2+Rotor_Gud_Wall_Thickness);
            }
            for(i=[0:360/2:360])
            {
                rotate(i+90)strut_up(x=Length/2,y=Width/2,apex=FC_Length/2-FC_Band,band=FC_Band,ir=Rotor_Diameter/2+Rotor_Gud_Wall_Thickness);
            }
            
        }
        // Holes on axes (center of Hyperb
        for(x=[-1:2:1])
        {
            for(y=[-1:2:1])
            {
                translate([Width/2*x,Length/2*y,0])
                rotate(atan2(-Width/2*x,Length/2*y)-180)
                rotate([roter_angle,0,0])
                rotor_gud_inner();
                
                translate([0,(FC_Length/2+bolt_r*3)*y,-Frame_Thickness/2])cylinder(r=bolt_r,h=Frame_Base_Hight*3,center=true);
            }
            translate([(FC_Width/2+bolt_r*3)*x,0,-Frame_Thickness/2])cylinder(r=bolt_r,h=Frame_Base_Hight*3,center=false);
        }
        
        translate([0,0,-.01])linear_extrude(height = Rotor_Gud_Frame_Thickness*2)panel_honeycomb([FC_Width*.85,FC_Length*.85],7,0,0); 
        /* AB old
        translate([0,0,-.01])linear_extrude(height = Rotor_Gud_Frame_Thickness*2)panel_honeycomb([FC_Width*.75,FC_Length*.75],6.75,1,0);
        */
        
    }
    
//    translate([0,0,Frame_Spacer_Hight+5])rotate([180,0,0])guard_mounter();
}

// sub component
module roter_cw()
{
    //AB:rotate([0,0,10])
    difference()
    {
        union()
        {
            translate([0,0,-(Motor_Cap+Frame_Thickness)])motor_cap_outer();
            if (Which_Parts == "print")
            {
                rotor_gud_frame(-Rotor_Frame_Angle);
            }
        }
        
        if (Which_Parts == "print")
        {
            translate([0,0,-(Motor_Cap+Frame_Thickness)])motor_cap_inner();
        }
        
    }
    
    difference()
    {
        translate([0,0,-Rotor_Frame_r/2-Rotor_Frame_h_offset])rotor_gud_outer(Rotor_Gud_h+Rotor_Frame_r/2+Rotor_Frame_h_offset,Rotor_Gud_Wall_Thickness);
        translate([0,0,-Rotor_Frame_r/2-Rotor_Frame_h_offset])rotor_gud_inner(Rotor_Gud_h+Rotor_Frame_r/2+Rotor_Frame_h_offset,Rotor_Gud_Wall_Thickness);
    }
    
}

module roter_ccw()
{
    
    difference(){
        union(){
            translate([0,0,-(Motor_Cap+Frame_Thickness)])motor_cap_outer();
            if (Which_Parts == "print"){
                rotor_gud_frame(Rotor_Frame_Angle);
            }
        }
        if (Which_Parts == "print"){
            translate([0,0,-(Motor_Cap+Frame_Thickness)])rotate()motor_cap_inner();
        }
    }
    
    difference(){
        translate([0,0,-Rotor_Frame_r/2-Rotor_Frame_h_offset])rotor_gud_outer(Rotor_Gud_h+Rotor_Frame_r/2+Rotor_Frame_h_offset,Rotor_Gud_Wall_Thickness);
        translate([0,0,-Rotor_Frame_r/2-Rotor_Frame_h_offset])rotor_gud_inner(Rotor_Gud_h+Rotor_Frame_r/2+Rotor_Frame_h_offset,Rotor_Gud_Wall_Thickness);
    }
}

module rotor_gud_frame(angle=Rotor_Frame_Angle){

    outer_r = Rotor_Diameter/2+Rotor_Gud_Wall_Thickness;
    rotor_gud_frame_upper_offset = -1;
    rotor_gud_frame_under_offset = 2.5;
    Rotor_Gud_Frame_Thickness = 1.25;
    
    intersection(){
        difference(){
            for (i=[1:Rotor_Frame_Num]){
                rotate([angle,0,i*360/Rotor_Frame_Num-90])translate([0,-Rotor_Gud_Frame_Thickness/2,-(Motor_Cap+Rotor_Frame_h_offset)])
                cube([outer_r,Rotor_Gud_Frame_Thickness,Motor_Cap+Rotor_Gud_h],center=false);
            }
            translate([0,0,-Motor_Cap-Rotor_Frame_h_offset+rotor_gud_frame_under_offset])
            rotate_extrude(convexity = 10, $fn = 100)hull()
            for(x=[0:1:1]){
                {
                    translate([Rotor_Diameter/2*x+scaled_motor_diameter/2+cap_wall_thickness+Rotor_Frame_r/2-.5, , 0])circle(r = Rotor_Frame_r/2, $fn = 100);
                }
            }
            translate([0,0,Rotor_Frame_r/2+rotor_gud_frame_upper_offset])hull()
            rotate_extrude(convexity = 10, $fn = 100)
            for(x=[0:1:1]){
                {
                    translate([outer_r-Rotor_Frame_r/2-Frame_Thickness/2, 0, 0])circle(r = Rotor_Frame_r/2, $fn = 100);
                }
            }
            translate([0,0,Rotor_Frame_r/2])cylinder(r=outer_r+.1,h=Rotor_Gud_h,center=false);
            translate([0,0,-(Motor_Cap+Frame_Thickness)])rotate([180,0,0])cylinder(r=Rotor_Diameter,h=Rotor_Gud_h,center=false);
        }
        union(){
            translate([0,0,-Motor_Cap+Rotor_Frame_r-Rotor_Frame_h_offset+rotor_gud_frame_under_offset])
            rotate_extrude(convexity = 5, $fn = 64)hull()
            for(y=[0:1:1]){
                {
                    translate([outer_r-Rotor_Frame_r/2, 0, 0])circle(r = Rotor_Frame_r/2, $fn = 100);
                }
            }
            translate([0,0,-Motor_Cap+Rotor_Frame_r-Rotor_Frame_h_offset+rotor_gud_frame_under_offset])cylinder(r=outer_r,h=Rotor_Gud_h*3,center=false);
            cylinder(r=outer_r-Rotor_Frame_r/2,h=Rotor_Gud_h*3,center=true);
        }
    }
}
module rotor_gud_outer(
     h=Rotor_Gud_h,
     cap_wall_thickness=Rotor_Gud_Wall_Thickness)
{
    //hexagons in guards
    hex_num = 18; //AB:50
    outer_height=h;
    hex_ring_mesh_R = outer_height; //AB:3.5
    outer_radius=Rotor_Diameter/2+Rotor_Gud_Wall_Thickness;
    
    
    difference()
    {
        translate([0,0,outer_height/2])
            cylinder(h=outer_height,r=outer_radius, center=true,$fn=256);
        if (Which_Parts == "print" && Guarde_mode == "mesh")
            {
            for (num=[0:outer_height/hex_ring_mesh_R]){
                for (i=[1:hex_num])
                {
//                    translate ([0,0,(outer_height/hex_ring_mesh_R)*num])rotate([0,0,i*360/hex_num])
//                        translate ([0,outer_radius,0])rotate([90,0,0])cylinder(r1=hex_ring_mesh_R/2,r2=0,h=Rotor_Gud_Wall_Thickness,$fn=6,center=false);
                    
                    translate ([0,0,(outer_height/hex_ring_mesh_R)*num+hex_ring_mesh_R/2-.75]) //AB:+hex_ring_mesh_R/8+.75])
                        rotate([0,0,i*360/hex_num+360/hex_num/2])
                        translate ([0,outer_radius,0])
                        rotate([90,0,0])
                        cylinder(r1=hex_ring_mesh_R/3,r2=0,h=outer_radius/2-.25,$fn=6,center=false);
                }
            }
        }
    }
    // Rotor Guard 
    //for(z=[-1:2:1])
    //{
    //  translate([0,0,outer_height/2*z+outer_height/2])
        translate([0,0,outer_height/2+outer_height/2])
        rotate_extrude(convexity = 10, $fn = 100)
        translate([outer_radius,0,0])
        circle(r=Rotor_Gud_Wall_Thickness);
    //}
}

module rotor_gud_inner(
     h=Rotor_Gud_h,
     cap_wall_thickness=Rotor_Gud_Wall_Thickness)
{
     inner_radius=Rotor_Diameter/2;
     inner_height=h;
	  union(){ 
	  translate([0,0,inner_height/2])
	       cylinder(h=inner_height+Frame_Thickness*3,r=inner_radius, center=true);
     }
}

module motor_cap_outer(
     h=Motor_Cap,
     inside_diameter=scaled_motor_diameter,
     cap_wall_thickness=cap_wall_thickness)
{
    cut_degree = 45;
    
     outer_radius=inside_diameter/2+cap_wall_thickness;
     outer_height=h+Frame_Thickness;
	       translate([0,0,outer_height/2])
	       cylinder(h=outer_height,r=outer_radius, center=true);
//    difference(){
     union()
     {
            translate([0,0,-1.5])
                    linear_extrude(height=1.5)
                    donutSlice(2.75,outer_radius,
                        270-cut_degree,270+cut_degree);
            translate([0,0,-Landing_leg_h])
                    linear_extrude(height=Landing_leg_h+.1)
                    donutSlice(scaled_motor_diameter/2,outer_radius,
                        270-cut_degree,270+cut_degree);
        }
//        for(r=[-1:2:1]){
//            hull()
//            for(z=[-1:2:1]){
//                translate([0,0,-Landing_leg_h/2*z-Landing_leg_h*.85])rotate([90,0,(85-cut_degree)*r])translate([2.25*r,0,0])
//                    cylinder(r=Landing_leg_h/2,h=outer_radius+100,center=true);
//            }
//        }
//    }
}

module motor_cap_inner(
     h=Motor_Cap,
     inside_diameter=scaled_motor_diameter,
     cap_wall_thickness=cap_wall_thickness,
     cap_wall_opening=cap_wall_opening)
{
    cut_degree = 45;
    
    inner_radius=inside_diameter/2;
    outer_radius=inside_diameter/2+cap_wall_thickness;
    inner_height=h;
    bevel_thickness=Motor_Cap_Inner_Bevel;
    
    union(){ 
        translate([0,0,inner_height/2+Frame_Thickness/2])
            cylinder(h=inner_height+Frame_Thickness+.1,r=inner_radius, center=true);
        translate([0,0,bevel_thickness/2])
            cylinder(h=bevel_thickness,r2=inner_radius,r1=inner_radius+bevel_thickness,center=true);
        translate([0,outer_radius,inner_height/2]) 
            cube([cap_wall_opening,2*outer_radius,inner_height+Frame_Thickness*3], center=true);
    }
    translate([0,0,-Landing_leg_h+Landing_leg_h/2])
            linear_extrude(height=Landing_leg_h/2+2.25/2)
            donutSlice(scaled_motor_diameter/2-.1,outer_radius+.1,
                cut_degree-70,255-cut_degree);
    for(r=[-1:2:1]){
        hull()
        for(z=[-1:2:1]){
            translate([0,0,-Landing_leg_h/2*z-Landing_leg_h*.85])rotate([90,0,(85-cut_degree)*r])translate([2.25*r,0,0])
                cylinder(r=Landing_leg_h/2,h=outer_radius+100,center=true);
        }
    }
}

module guard_centre(w=FC_Width,l=FC_Length,h=Frame_Thickness,band=FC_Band){

    d=sqrt(l*l+w*w);
    
    difference()
    {
        union()
        {
            translate([0,0,h/2])
            difference()
            {
                union()
                {
                    hull()for(x=[-1:2:1])
                    {
                        for(y=[-1:2:1])
                        {
                            translate([(w+bolt_r*1.41)/2*x,(l+bolt_r*1.41)/2*y,0])cylinder(r=bolt_r*3,h=h,center=true);
                        }
                    }
                    for(x=[-1:2:1])
                    {
                        for(y=[-1:2:1])
                        {
                            
                            translate([(w+bolt_r*1.41)/2*x,(l+bolt_r*1.41)/2*y,h/2])
                            {
                                rotate([180,0,0])cylinder(r=bolt_r*2.5,h=Frame_Base_Hight+h,center=false);
                            }
                            
                            // lateral beam
                            translate([0,(l+bolt_r*1.41)/2*y,-h*.75])
                            {
//                                difference(){
                                    cube([w,bolt_r*2,h*2.5],center=true);
//                                    cube([w-bolt_r*9,bolt_r*3,h*5],center=true);
//                                }
                            }
                            
                            translate([0,(l/2+bolt_r*3)*y,-h/2])
                            {
                                cylinder(r=bolt_r*2,h=5.0,center=false);
                                %translate([0,0,5])color("White")cylinder(r=4.6/2,h=Frame_Spacer_Hight,$fn=6);
                            }
                            
                        }
                        // lateral beam                        
                        translate([(w+bolt_r*1.41)/2*x,0,-h*.75])
                        {
//                            difference(){
                                cube([bolt_r*2,w,h*2.5],center=true);
//                                cube([bolt_r*3,w-bolt_r*9,h*5],center=true);
//                            }
                        }
                        
                        // Cylinder in center of Hyperbol on Frame
                        translate([(w/2+bolt_r*3)*x,0,-h/2])
                        {
                            cylinder(r=bolt_r*2,h=5.0,center=false);
                             %translate([0,0,5])color("White")cylinder(r=4.6/2,h=Frame_Spacer_Hight,$fn=6);
                        }
                        
                        
                    }
                }
                // notch in the four angles
                //translate([0,0,-Frame_Base_Hight-h+1.5])cube([FC_Width+.25,FC_Length+.25,1.35],center=true);
                translate([0,0,-Frame_Base_Hight-h+1.5])cube([FC_Width+.25,FC_Length+.25,1.6],center=true);
                %translate([0,0,-Frame_Base_Hight-h+1.5-.15])color("Blue")cube([FC_Width+.25,FC_Length+.25,1.6],center=true);
                
            }
        }
        
        // Holes of 4 external long-distancers
        for(x=[-1:2:1])
        {
            for(y=[-1:2:1])
            {
                translate([(w+bolt_r*1.41)/2*x,(l+bolt_r*1.41)/2*y,-.1])cylinder(r=bolt_r,h=Frame_Base_Hight*3,center=true);
//                translate([0,(l/2+bolt_r*3)*y,-h/2])cylinder(r=bolt_r,h=Frame_Base_Hight*3,center=true);
            }
//            translate([(w/2+bolt_r*3)*x,0,-h/2])cylinder(r=bolt_r,h=Frame_Base_Hight*3,center=false);
        }
        
     }
}

module FCU_support(w=FC_Width,l=FC_Length,h=Frame_Thickness,band=FC_Band)
{
    
    difference()
    {
        union()
        {
            translate([0,0,h/2-Frame_Base_Hight-h-.5])
            difference()
            {
                union()
                {
                    intersection()
                    {
                        union()
                        {
                            translate([0,0,146])rotate([90,0,45]){
                                difference(){
                                cylinder(r=150,h=h*1.25,center=true);
                                translate([0,h,0])cylinder(r=150,h=h*3,center=true);
                                }
                            }
                            translate([0,0,146])rotate([90,0,-45]){
                                difference(){
                                    cylinder(r=150,h=h*1.25,center=true);
                                    translate([0,h,0])cylinder(r=150,h=h*3,center=true);
                                }
                            }
                        }
                        translate([0,0,0])cube([FC_Width+.25,FC_Length+.25,10],center=true);
                    }
                    for(x=[-1:2:1]){
                        for(y=[-1:2:1]){
                            translate([(w+bolt_r*1.41)/2*x,(l+bolt_r*1.41)/2*y,h/2]){
                                rotate([180,0,0])cylinder(r=bolt_r*2.5,h=h+1,center=false);
                            }
                            // beam
                            translate([0,(l+bolt_r*1.41)/2*y,-h*.4]){
                                cube([w,bolt_r*2,h*1.5],center=true);
                            }
                        }
                        // beam
                        translate([(w+bolt_r*1.41)/2*x,0,-h*.4]){
                            cube([bolt_r*2,w,h*1.5],center=true);
                        }
                    }
                }
            }
        }
        for(x=[-1:2:1]){
            for(y=[-1:2:1]){
                translate([(w+bolt_r*1.41)/2*x,(l+bolt_r*1.41)/2*y,-.1])cylinder(r=bolt_r+.25,h=Frame_Base_Hight*3,center=true);
            }
        }
    } 
}

module guard_mounter(){
    // Guard Mounter
    guard_mount_shaft_r = 2.0;
    cut_degree = 30;
    mount_shaft_h = -6.5;
    mount_shaft_l = 40;
    
        w=FC_Width;
        l=FC_Length;
        h=Frame_Thickness;
    union(){
        difference(){
            union(){
                for(x=[-1:2:1]){
                    for(y=[-1:2:1]){
                        hull(){
                            translate([(w*.8)/2*x,(l*.8)/2*y,-h/2+mount_shaft_h/2])
                                cylinder(r=bolt_r*1.25,h=1.5,center=true);
                            translate([(w/2+bolt_r*3)*x,0,0])
                                rotate([180,0,0])cylinder(r=bolt_r*2,h=2,center=false);
                        }
                        hull(){
                            translate([(w*.8)/2*x,(l*.8)/2*y,-h/2+mount_shaft_h/2])
                                cylinder(r=bolt_r*1.25,h=1.5,center=true);
                            translate([0,(l/2+bolt_r*3)*y,0])
                                rotate([180,0,0])cylinder(r=bolt_r*2.,h=2,center=false);
                        }
                        hull(){
                            translate([(w*.8)/2*x,(l*.8)/2*y,-h/2+mount_shaft_h/2])
                                cylinder(r=bolt_r*1.25,h=1.5,center=true);
                        translate([(w/3)*x,0,mount_shaft_h])
                            rotate([0,90,0])cylinder(r=guard_mount_shaft_r/2+.9,h=2,center=true);
                        }
                        hull(){
                            translate([(w*.8)/2*x,(l*.8)/2*y,-h/2+mount_shaft_h/2])
                                cylinder(r=bolt_r*1.25,h=1.5,center=true);
                            translate([0,(l/3)*y,mount_shaft_h])
                                rotate([90,0,0])cylinder(r=guard_mount_shaft_r/2+.9,h=2,center=true);
                        }
                    }
                }
            }
            for(x=[-1:2:1]){
                for(y=[-1:2:1]){
                        translate([(w/2+bolt_r*3)*x,0,-2])rotate([180,0,0]){
                            cylinder(r=bolt_r*2.3,h=2,center=false);
                            cylinder(r=bolt_r,h=10,center=true);
                        }
                        translate([0,(l/2+bolt_r*3)*y,-2])rotate([180,0,0]){
                            cylinder(r=bolt_r*2.3,h=2,center=false);
                            cylinder(r=bolt_r,h=10,center=true);
                        }
                }
            }
            for(z=[0:90:360]){
                rotate([0,90,z])translate([-mount_shaft_h,0,0])
                cylinder(r=guard_mount_shaft_r/2+.91,h=mount_shaft_l/2,center=false);
            }
            
        }
        difference()
        {
            union()
            {
                for(z=[90:180:270])
                {
                    rotate([0,90,z])translate([-mount_shaft_h,0,0])
                    linear_extrude(height=mount_shaft_l/2)
                    donutSlice(guard_mount_shaft_r/2,guard_mount_shaft_r/2+.9,
                        0+cut_degree/2,360-cut_degree/2);
                }
            }
            rotate([0,90,180])translate([-mount_shaft_h,0,0])
            cylinder(r=guard_mount_shaft_r/2+.91,h=mount_shaft_l/2,center=true);
        }
        for(z=[0:180:360])
        { 
            rotate([0,90,z])translate([-mount_shaft_h,0,0])
            linear_extrude(height=mount_shaft_l/2)
            donutSlice(guard_mount_shaft_r/2,guard_mount_shaft_r/2+1,
                0+cut_degree/2,360-cut_degree/2);
        }
    }
}

// sub module
module strut(x=Width/2,y=Length/2,apex=FC_Length/2-FC_Band,band=FC_Band,ir=scaled_motor_diameter/2+cap_wall_thickness){

     r=ir-band;
     //This hyperbola meets the motor at the outside at point (x0,y0).
     //Let the motor mount have radius r and be centered at (x,y),
     //then (y0-y)/(x0-x)= -(x/y) and (x0-x)^2+(y0-y)^2=r^2.
     //The solution for y0 and x0 are as follows:
     x0 = -y*r/sqrt(x*x+y*y)+x;
     y0 = x*r/sqrt(x*x+y*y)+y;
     aa=apex*x0/sqrt(y0*y0-apex*apex);
     
     //Here we use the same hyperbola, translated vertically by v, so
     //that the width of the struts are FC_Band where they meet the
     //motors.
     theta=atan2(y,x);
     deltaX=-band*cos(theta);
     deltaY=band*sin(theta);

     ix0 = x0+deltaX;
     iy0 = y0+deltaY;
     v=iy0-apex*sqrt(1+ix0*ix0/(aa*aa));
    
     intersection(){
          difference(){
               linear_extrude(height=Frame_Thickness,convexity=10)hyperbola(a=aa,b=apex,h=max(Width,Length)*5);	       
               scale([1,1,1.1])translate([0,0,-0.1])linear_extrude(height=Frame_Thickness,convexity=10)hyperbola(a=aa,b=apex,v=v,h=max(Width,Length)*5);
          }
          translate([0,0,Frame_Thickness/2])
               cube([2*x+ir,2*y+ir,Frame_Thickness],center=true);
     }
}
module strut_up(x=Width/2,y=Length/2,apex=FC_Length/2-FC_Band,band=FC_Band,ir=scaled_motor_diameter/2+cap_wall_thickness){

     r=ir-band;
     //This hyperbola meets the motor at the outside at point (x0,y0).
     //Let the motor mount have radius r and be centered at (x,y),
     //then (y0-y)/(x0-x)= -(x/y) and (x0-x)^2+(y0-y)^2=r^2.
     //The solution for y0 and x0 are as follows:
     x0 = -y*r/sqrt(x*x+y*y)+x;
     y0 = x*r/sqrt(x*x+y*y)+y;
     aa=apex*x0/sqrt(y0*y0-apex*apex);
     
     //Here we use the same hyperbola, translated vertically by v, so
     //that the width of the struts are FC_Band where they meet the
     //motors.
     theta=atan2(y,x);
     deltaX=-band*cos(theta);
     deltaY=band*sin(theta);

     ix0 = x0+deltaX;
     iy0 = y0+deltaY;
     v=iy0-apex*sqrt(1+ix0*ix0/(aa*aa));
    
     intersection(){
          difference(){
               union(){
                    linear_extrude(height=Rotor_Gud_Frame_Thickness,convexity=10)hyperbola(a=aa,b=apex,h=max(Width,Length));	  
                    translate([0,0,0])linear_extrude(height=Rotor_Gud_Frame_Thickness*2.5,convexity=10)
                        hyperbola(a=aa+Rotor_Gud_Frame_Thickness/2,b=apex+Rotor_Gud_Frame_Thickness*2,h=max(Width,Length));  
               }   
               scale([1,1,1.1])translate([0,0,-0.1])linear_extrude(height=Rotor_Gud_Frame_Thickness*3,convexity=10)
                    hyperbola(a=aa,b=apex,v=v,h=max(Width,Length)*5);
          }
          translate([0,0,Rotor_Gud_Frame_Thickness/2])
               cube([2*x+ir,2*y+ir,Rotor_Gud_Frame_Thickness*5],center=true);
     }
}
module hyperbola(a=1,b=1,v=0,h=500){
    
    translate([0,v,0])scale([a,b,1])
        projection(cut=true)
        translate([0,0,1])
        rotate([-90,0,0])
        cylinder(h=h/35,r1=0,r2=h/35);
}
module panel_honeycomb( arrDim=[100,50], thickness=5, d_off=0, center=true )
{
  d = thickness;
  thickness = thickness/3;

    for( off=[[0,0],[(d/2)+thickness,(d/3)+(thickness/2)]] )
    {
      xoff = off[0];
      yoff = off[1];
    
      //echo("yoff=", yoff);
      
      for( y=[0 : d : arrDim[1]/2] )
      {
        {
          for( x=[0 : d+thickness+thickness : arrDim[0]/2] )
          {
            {
              translate([x+xoff,y+yoff,0]) circle(d=d-d_off, $fn=6,center=true);
              translate([-(x+xoff),y+yoff,0]) circle(d=d-d_off, $fn=6,center=true);
            }
          }
    
          for( x=[0 : d+thickness+thickness : arrDim[0]/2] )
          {
            {
              translate([x+xoff,-(y+yoff),0]) circle(d=d-d_off, $fn=6,center=true);
              translate([-(x+xoff),-(y+yoff),0]) circle(d=d-d_off, $fn=6,center=true);
            }
          }
        }
      }
    }
}
module donutSlice(innerSize,outerSize, start_angle, end_angle) 
{   
    difference()
    {
        pieSlice(outerSize, start_angle, end_angle);
        if(len(innerSize) > 1) ellipse(innerSize[0]*2,innerSize[1]*2); //(innerSize, start_angle-1, end_angle+1);
        else circle(innerSize);
    }
}
module pieSlice(size, start_angle, end_angle) //size in radius(es)
{	
    rx = ((len(size) > 1)? size[0] : size);
    ry = ((len(size) > 1)? size[1] : size);
    trx = rx* sqrt(2) + 1;
    try = ry* sqrt(2) + 1;
    a0 = (4 * start_angle + 0 * end_angle) / 4;
    a1 = (3 * start_angle + 1 * end_angle) / 4;
    a2 = (2 * start_angle + 2 * end_angle) / 4;
    a3 = (1 * start_angle + 3 * end_angle) / 4;
    a4 = (0 * start_angle + 4 * end_angle) / 4;
    if(end_angle > start_angle)
        intersection() {
		if(len(size) > 1)
        	ellipse(rx*2,ry*2);
		else
			circle(rx);
        polygon([
            [0,0],
            [trx * cos(a0), try * sin(a0)],
            [trx * cos(a1), try * sin(a1)],
            [trx * cos(a2), try * sin(a2)],
            [trx * cos(a3), try * sin(a3)],
            [trx * cos(a4), try * sin(a4)],
            [0,0]
       ]);
    }
}
module ellipse(width, height) {
  scale([1, height/width, 1]) circle(r=width/2);
}