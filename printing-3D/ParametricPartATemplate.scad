// title      : RoboGen parametric A part
// author     : Alice Concordel

partA(0.03,0); //the first parameter is the length of the joint, the second parameter is the angle beta (only for non-planar robots)

module partA(length, beta)
{
//plate parameters
	lengthSlide = 34;
	widthSlide = 34;
	heightSlide = 1.5;

	radiusHole = 1;

//wire hole parameters
	lengthWire = 3;
	widthWire = 9;
	heightWire = heightSlide;

//shaft part parameters
	lengthShaft = 20;
	widthShaft = 9;

// apply transformations to the parameter
length = length*1000; //convert to mm
	union()
	{
		difference()
		{
			// starting plate
			cube([lengthSlide,widthSlide,heightSlide]);
		
			union()
			{
			// chamfer the edges
				rotate ([0,-45,0]) cube([3,widthSlide,3]);
				rotate([45,0,0]) cube([lengthSlide,3,3]);
				translate([lengthSlide,0,0]) rotate([0,-45,0]) cube([3,widthSlide,3]);
				translate([0,widthSlide,0]) rotate([45,0,0]) cube([lengthSlide,3,3]);

			// holes in the plate
					//round
				translate([4.5,4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([lengthSlide-4.5,4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([4.5,widthSlide-4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
				translate([lengthSlide-4.5,widthSlide-4.5,-0.5]) cylinder(r = radiusHole, h = heightSlide+1, $fn = 30);
					
					//rectangular holes (only present if the shaft is not rotated)
				if(beta == 0){
				translate([7-lengthWire/2,widthSlide/2-widthWire/2,-0.5]) cube([lengthWire,widthWire,heightSlide+1]);
				translate([lengthSlide-7-lengthWire/2,widthSlide/2-widthWire/2,-0.5]) cube([lengthWire,widthWire,heightSlide+1]);
				}
			}
		}

	//the shaft
		//========================== beta paramter below: rotates the whole thing if needed
		translate([lengthSlide/2,widthSlide/2,0]) rotate([0,0,beta]) translate([-lengthSlide/2,-widthSlide/2,0]) 
	union()
	{
		//main long part 
		//========================== length parameter below
			difference()
			{
			translate([lengthSlide/2-widthShaft/2,widthSlide/2-lengthShaft/2,0]) cube([widthShaft,lengthShaft,length+heightSlide]);
			translate([lengthSlide/2-(widthShaft-4)/2,widthSlide/2-(lengthShaft-4)/2,heightSlide]) cube([widthShaft-4,lengthShaft-4,length+0.5]);
			}

		//chamfer the edges of the shaft base
			difference()
			{
				translate([lengthSlide/2-(widthShaft+3)/2,widthSlide/2-(lengthShaft+3)/2]) cube([widthShaft+3,lengthShaft+3,3]);
				union()
				{
				translate([lengthSlide/2-widthShaft/2-1.5,0,1.5]) rotate ([0,-45,0]) cube([1.5*sqrt(2),widthSlide,1.5*sqrt(2)]);
				translate([0,widthSlide/2-lengthShaft/2-1.5,1.5]) rotate([45,0,0]) cube([lengthSlide,1.5*sqrt(2),1.5*sqrt(2)]);
				translate([lengthSlide/2+widthShaft/2+1.5,0,1.5]) rotate([0,-45,0]) cube([1.5*sqrt(2),widthSlide,1.5*sqrt(2)]);
				translate([0,widthSlide/2+lengthShaft/2+1.5,1.5]) rotate([45,0,0]) cube([lengthSlide,1.5*sqrt(2),1.5*sqrt(2)]);
				}
			}
		}
	}
}

