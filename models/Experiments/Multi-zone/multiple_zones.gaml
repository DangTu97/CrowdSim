/**
* Name: multiplezones
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model multiplezones
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../Continuum_Crowds/high_density_zone.gaml'

/* Insert your model definition here */

global {
	float step <- STEP;
//	geometry shape <- envelope(square(80#m));
	geometry shape <- envelope(rectangle(80, 40));
//	geometry shape <- envelope(rectangle(40, 60));
	
	geometry space <- shape;
	
	init {
		MODEL_NAME <- 'multi_zone';
		
		// operational level
		create walking_directly number: 1;
		create following number: 1;
		
		point group_target <- {1.0, 29.5};
		
		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 30;
			CELL_SIZE_X <- 30 / GRID_WIDTH;
		    CELL_SIZE_Y <- 30 / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;
			boundary <- [5.0, 35.0, 0, 30];
			group_goals <- [1::[[0, 29]]];
			do init_state;
		}
		
		create high_density_zone {
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 30;
			CELL_SIZE_X <- 30 / GRID_WIDTH;
		    CELL_SIZE_Y <- 30 / GRID_HEIGHT;
		 	CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			nb_groups <- 1;
			boundary <- [45.0, 75, 0, 30];
			group_goals <- [1::[[29, 0]]];
			do init_state;
		}
		
		create individual number: 6000 {
			color <- #blue;
//			location <- any_location_in(space);
			location <- one_of([{rnd(5.0, 35.0), rnd(0.0, 30.0)}, {rnd(45.0, 75.0), rnd(0.0, 30.0)}]);
			target <- group_target;
			velocity <- (target - location) * rnd(0.3, 1.4)/ norm(target - location);
			group_id <- 1;
			my_operational_behavior <- first(following);
			my_zone <- location.x < 35.0 ? high_density_zone(0): high_density_zone(1);
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
