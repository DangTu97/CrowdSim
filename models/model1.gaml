/**
* Name: model1
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model model1
import 'global_variables.gaml'
import 'Individual/individual.gaml'
import 'Continuum_Crowds/high_density_zone.gaml'

/* Insert your model definition here */

global {
	float step <- STEP;
	geometry shape <- envelope(50#m);
	geometry space <- shape;
	
	init {
		point group_target <- {25.5, 25.5};

		// operational level
		create walking_directly number: 1;
		create following number: 1;
		
		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 30;
			CELL_SIZE_X <- 30 / GRID_WIDTH;
		    CELL_SIZE_Y <- 30 / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;
			boundary <- [10.0, 40, 10, 40];
			float x_min <- boundary[0];
			float y_min <- boundary[2];
			group_goals <- [1::[[int((group_target.x - x_min) / CELL_SIZE_X), int((group_target.y - y_min) / CELL_SIZE_Y)]]];
			do init_state;
		}
		
		create individual number: 2000 {
			color <- #blue;
			location <- any_location_in(space);
			target <- group_target;
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			my_operational_behavior <- first(walking_directly);
			my_zone <- first(high_density_zone);
		}

	}

	reflex save_pedestrian_locations when: cycle = 100 and false {
		ask individual {
			save [location.x, location.y] type: 'csv' to: '../Includes/position.csv' rewrite: false;
		}
	}
	
}

experiment exp parallel: true {
	output {
		display my_display {
			species individual;
		}
	}
}