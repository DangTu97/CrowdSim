/**
* Name: model0
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model model0
import 'global_variables.gaml'
import 'Individual/individual.gaml'
import 'Continuum_Crowds/high_density_zone.gaml'

/* Insert your model definition here */

global {
	float step <- STEP;
	geometry shape <- envelope(square(30#m));
//	geometry shape <- envelope(rectangle(60, 40));
//	geometry shape <- envelope(rectangle(40, 60));
	
	geometry space <- shape;
	
	init {
		
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
			boundary <- [0.0, 30, 0, 30];
			group_goals <- [1::[[0, 29]]];
			obstacle_cells <- [[15,15], [15,16], [15,17]];
			do init_state;
		}
		
		create individual number: 3000 {
			color <- #blue;
			location <- any_location_in(space);
			target <- group_target;
			velocity <- (target - location) * rnd(0.3, 1.4)/ norm(target - location);
			group_id <- 1;
			my_operational_behavior <- first(following);
			my_zone <- first(high_density_zone);
		}
		
	}

}

grid my_cell width: 30 height: 30 {
	
	aspect default {
		draw shape color: #white border: #black;
		draw line([{0, 0}, {first(high_density_zone).group_gradient_field_WE_SN[1][grid_x][grid_y][0], 
			first(high_density_zone).group_gradient_field_WE_SN[1][grid_x][grid_y][1]
		}]) at: location color: #red;
	}
}

experiment exp parallel: true {
	output {
		display my_display {
			species my_cell;
			species individual;
		}
	}
}

