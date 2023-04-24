/**
* Name: genmap
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model evacuation
import '../global_variables.gaml'
import '../Individual/individual.gaml'
import '../Continuum_Crowds/high_density_zone.gaml'
import '../env.gaml'
import '../helpers/detection.gaml'

/* Insert your model definition here */

global {
	float step <- STEP;
	
	float road_length <- 30.0 #m;
	float road_width <- 20.0 #m;
	float size <- 100.0 #m;
	float door_width <- 2.0 #m;
	float exit_width <- 3.0 #m;
	
	geometry shape <- envelope(size + road_length + 2);
	
	point group_target <- {129.5, 50.0};
	
	init {
		// left
		create building {
			shape <- polygon([{0, 0}, {0.2 * size, 0}, {0.2 * size, 0.5 * size}, {0, 0.5 * size}]);
		}
		create building {
			shape <- polygon([{0, 0.5 * size}, {0.2 * size, 0.5 * size}, {0.2 * size, size}, {0, size}]);
		}
		
		// up 
		create building {
			shape <- polygon([{0.2 * size, 0}, {0.6 * size, 0}, {0.6 * size, 0.2 * size}, {0.2 * size, 0.2 * size}]);
		}
		create building {
			shape <- polygon([{0.6 * size, 0}, {size, 0}, {size, 0.2 * size}, {0.6 * size, 0.2 * size}]);
		}
		
		// down 
		create building {
			shape <- polygon([{0.2 * size, 0.8 * size}, {0.6 * size, 0.8 * size}, {0.6 * size, size}, {0.2 * size, size}]);
		}
		create building {
			shape <- polygon([{0.6 * size, 0.8 * size}, {size, 0.8 * size}, {size, size}, {0.6 * size, size}]);
		}
		
		//wall
		create wall {
			shape <- polyline([{size, 0.2 * size}, {size, 0.5 * size - road_width / 2}, {size + road_length, 0.5 * size - road_width / 2}]);
		}
		
		create wall {
			shape <- polyline([{size, 0.8 * size}, {size, 0.5 * size + road_width / 2}, {size + road_length, 0.5 * size + road_width / 2}]);
		}
		
		// wall exit up
		create wall {
			shape <- polyline([{size + road_length, size / 2 - road_width / 2}, {size + road_length, size / 2 - exit_width / 2}]);
		}
		
		// wall exit down
		create wall {
			shape <- polyline([{size + road_length, size / 2 + road_width / 2}, {size + road_length, size / 2 + exit_width / 2}]);
		}
		
		// left door 
		create door {
			shape <- polyline([{0.2 * size, 0.35 * size - door_width / 2}, {0.2 * size, 0.35 * size + door_width / 2}]) + 1.0;
		}
		create door {
			shape <- polyline([{0.2 * size, 0.65 * size - door_width / 2}, {0.2 * size, 0.65 * size + door_width / 2}]) + 1.0;
		}
		
		// up
		create door {
			shape <- polyline([{0.4 * size - door_width / 2, 0.2 * size}, {0.4 * size + door_width / 2, 0.2 * size}]) + 1.0;
		}
		create door {
			shape <- polyline([{0.8 * size - door_width / 2, 0.2 * size}, {0.8 * size + door_width / 2, 0.2 * size}]) + 1.0;
		}
		
		// down
		create door {
			shape <- polyline([{0.4 * size - door_width / 2, 0.8 * size}, {0.4 * size + door_width / 2, 0.8 * size}]) + 1.0;
		}
		create door {
			shape <- polyline([{0.8 * size - door_width / 2, 0.8 * size}, {0.8 * size + door_width / 2, 0.8 * size}]) + 1.0;
		}
		
		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 20;
			CELL_SIZE_X <- 30 / GRID_WIDTH;
		    CELL_SIZE_Y <- 20 / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;
			boundary <- [size, size + road_length, size/2 - road_width/2, size/2 + road_width/2];
			float x_min <- boundary[0];
			float y_min <- boundary[2];
			group_goals <- [1::[ [29, 9], [29, 10], [29, 11] ]];
			do init_state;
		}
		
		// operational level
		create walking_directly number: 1;
		create following number: 1;
			
	
		float X_min <- size;
		float X_max <- size + road_length;
		float Y_min <- size/2 - road_width/2;
		float Y_max <- size/2 + road_width/2;
		geometry ok <- polygon([{X_min, Y_min}, {X_max, Y_min}, {X_max, Y_max}, {X_min, Y_max}]);
		
		create individual number: 1000 {
			color <- #blue;
			location <- any_location_in(one_of(door));
			target <- group_target;
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
//			local_target <- first(polyline([{size, 0.5 * size - road_length / 2}, {size, 0.5 * size + road_length / 2}]) closest_points_with self.location);
			location <- any_location_in(ok);
			local_target <- {1.01 * size, rnd(0.5 * size - road_width / 2, 0.5 * size + road_width / 2)};
			my_operational_behavior <- first(following);
			my_zone <- first(high_density_zone);
		}
		
	}
	
	reflex init_people {
		create individual number: 5 {
			color <- #blue;
			location <- any_location_in(one_of(door));
			target <- group_target;
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
//			local_target <- first(polyline([{size, 0.5 * size - road_length / 2}, {size, 0.5 * size + road_length / 2}]) closest_points_with self.location);
			local_target <- {1.01 * size, rnd(0.5 * size - road_width / 2, 0.5 * size + road_width / 2)};
			my_operational_behavior <- first(walking_directly);
			my_zone <- first(high_density_zone);
		}
	}
}

species wall {
	aspect default {
		draw shape color: #grey;
	}
}

species door {
	aspect default {
		draw shape color: #white border: #black;
	}
}

experiment exp parallel: true {
	output {
		display my_display {
			species building;
			species wall;
			species door;
			species individual;
		}
	}
}