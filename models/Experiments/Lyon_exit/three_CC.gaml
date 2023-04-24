/**
* Name: allCC
* Based on the internal empty template. 
* Author: hdang
* Tags: 3 CC models
*/


model threeCC
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../Continuum_Crowds/high_density_zone.gaml'
import '../../env.gaml'
/* Insert your model definition here */

global {
	float step <- STEP;
	
	file building_shape_file <- shape_file('../../includes/place_des_Terreaux/ok.shp');
	file road_shape_file <- shape_file('../../includes/place_des_Terreaux/lines.shp');
	file center_shape_file <- shape_file('../../includes/place_des_Terreaux/center.shp');
	file init_space_shape_file <- shape_file('../../includes/place_des_Terreaux/init_space.shp');
	
	file nav_mesh_shape_file <- shape_file('../../includes/gen_map_official/mesh.shp');
	file mesh_centroid_shape_file <- shape_file('../../includes/gen_map_official/node.shp');
	
	geometry shape <- envelope(building_shape_file);
	geometry space <- copy(shape);
	point group_target <- {60, 60};
	
	float LENGTH_X;
	float LENGTH_Y;
	
	int nb_agents <- 6000;
	
	init {
		MODEL_NAME <- 'all_CC';
		MAXIMUM_SPEED <- 1.0 #m/#s;
		MINIMUM_SPEED <- 0.1 #m/#s;
		
		// ENVIRONMENTS
		create building from: building_shape_file;
		create center from: center_shape_file;
		
		create nav_mesh from: nav_mesh_shape_file;
		create mesh_centroid from: mesh_centroid_shape_file;
		
		LENGTH_X <- first(heat_map).shape.points[2].x;
		LENGTH_Y <- first(heat_map).shape.points[1].y;
		
		// HIGH-DENSITY ZONE 0
		create high_density_zone {
			//GRID_WIDTH <- 30;
			//GRID_HEIGHT <- 30;
			//group_goals <- [1::[[0, 29]]];
			
			GRID_WIDTH <- 60;
			GRID_HEIGHT <- 60;
			group_goals <- [1::[[0, 59]]];
			
			nb_groups <- 1;
			boundary <- [15.7, 76.5, 31.0, 70.5];
			float x_min <- boundary[0];
			float x_max <- boundary[1];
			float y_min <- boundary[2];
			float y_max <- boundary[3];
			CELL_SIZE_X <- (x_max - x_min) / GRID_WIDTH;
			CELL_SIZE_Y <- (y_max - y_min) / GRID_HEIGHT;
			CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			
			loop i from: 0 to: GRID_WIDTH - 1 {
				loop j from: 0 to: GRID_HEIGHT - 1 {
					bool check <- false;
					loop b over: building {
						// extend boundary of building3 to avoid unrealistic phenomena
						geometry b_shape <- b.name = "building3" ? b.shape + CELL_SIZE_X / 2 : b.shape;
						
						if {x_min + CELL_SIZE_X * i, y_min + CELL_SIZE_Y * j} overlaps b_shape {
							check <- true;
						}
					}
					if check {
						obstacle_cells <- obstacle_cells + [[i, j]];
					}
				}
			}
			
			do init_state;
			exit_meshes <- [1::nav_mesh(30)];
		}
		
		// HIGH-DENSITY ZONE 1
		create high_density_zone {
			GRID_WIDTH <- 20;
			GRID_HEIGHT <- 20;
			nb_groups <- 1;
			boundary <- [0.0, 22.8, 64.5, 70.5];
			float x_min <- boundary[0];
			float x_max <- boundary[1];
			float y_min <- boundary[2];
			float y_max <- boundary[3];
			CELL_SIZE_X <- (x_max - x_min) / GRID_WIDTH;
			CELL_SIZE_Y <- (y_max - y_min) / GRID_HEIGHT;
			CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			group_goals <- [1::[[0, 10]]];
			do init_state;
			exit_meshes <- [1::nav_mesh(30)];
		}
		
		// HIGH-DENSITY ZONE 2
		create high_density_zone {
			GRID_WIDTH <- 20;
			GRID_HEIGHT <- 20;
			nb_groups <- 1;
			boundary <- [15.9, 22.8, 64.5, 94.5];
			float x_min <- boundary[0];
			float x_max <- boundary[1];
			float y_min <- boundary[2];
			float y_max <- boundary[3];
			CELL_SIZE_X <- (x_max - x_min) / GRID_WIDTH;
			CELL_SIZE_Y <- (y_max - y_min) / GRID_HEIGHT;
			CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			group_goals <- [1::[[10, 19]]];
			do init_state;
			exit_meshes <- [1::nav_mesh(30)];
		}
		
		create following number: 1;
		
		create individual number: nb_agents {
			color <- #blue;
			target <- {8.0, 85.5};
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			
			group_id <- 1;
			
			my_operational_behavior <- first(following);
			
			my_tactical_behavior <- first(best_neighbor_mesh);
			
			location <- any_location_in(first(center));
			
			is_at_dense_region <- true;
			color <- #red;
			my_zone <- first(high_density_zone);
		}

	}
	
	reflex measure_simulation_time  when: mod(cycle, 100) = 0 {
		write #now;
	}
	
	reflex write_evacuation_time when: length(individual) = 0 {
		write("Evacuation Time: " + time * step);
		do pause;
	}
	
	reflex draw_heat_map when: true {

		ask heat_map {
			nb_agents <- 0;
			speed_sum <- 0.0;
		}
		
		loop ag over: individual {
			if dead(ag) = false {
				int idx <- int(ag.location.x / LENGTH_X);
				int idy <- int(ag.location.y / LENGTH_Y);
				
				if heat_map[idx, idy] != nil {
					ask heat_map[idx, idy] {
						nb_agents <- nb_agents + 1;
						speed_sum <- speed_sum + norm(ag.velocity);
					}
				}
				
			}
		}
		
		ask heat_map {
			do update_fd;
		}
		
		ask heat_map {
			do spread_out;
		}
		
		ask heat_map {
			do define_color;
		}
	}
	
	reflex count_and_save_exit_pedestrian when: true {
		write nb_exit_pedestrians;
		save [cycle, nb_exit_pedestrians] type: csv to: "data/three_CC/sim4_" + string(nb_agents) + ".txt" rewrite: false;
	}
	
	reflex calibrate when: true {
		if cycle < 600 {
			MINIMUM_SPEED <- 0.1 #m/#s;
		} else {
			MINIMUM_SPEED <- 0.22 #m/#s;
		}
		
		ask individual {
			if (location.x < 6.5 or location.y > 84.5) {
				do die;
			}
		}
	}
	
}

grid heat_map width: 60 height: 60 {
	float speed_sum;
	float average_speed;
	float density;
	int nb_agents;
	rgb my_color;
	
	action define_color {
		
		if density < 0.5 {
			float t <- density / 0.5;
			my_color <- rgb(0, 200 + (255 - 200) * t, 255);
		} else if density < 1.0 {
			float t <- (1.0 - density) / 0.5; // t: 1 => 0
			my_color <- rgb(0, 255, int(255 * t ));
		} else if density < 2.0 {
			float t <- (density - 1.0) / 1.0; // t: 0 => 1
			my_color <- rgb( int(255 * t) , 255, 0);
		} else {
			float t <- (density - 2.0) / 4.0;
			my_color <- rgb(255, 255 * (1 - t), 0);
		}
		
	}
	
	action spread_out {
		int N <- length(neighbors);
		float old_density <- density;
		
		density <- old_density / (N + 1);
	
		ask neighbors {
			self.density <- self.density + old_density / (N + 1);
		}

	}
	
	action update_fd {
		density <- nb_agents / (LENGTH_X * LENGTH_Y);
		average_speed <- nb_agents = 0 ? 0.0 : speed_sum / nb_agents;
	}

	aspect default {
		draw shape color: my_color border: my_color wireframe: false;
		
		//draw line([{0, 0}, {first(high_density_zone).group_gradient_field_WE_SN[2][grid_x][grid_y][0], 
		//					first(high_density_zone).group_gradient_field_WE_SN[2][grid_x][grid_y][1]}]) at: location color: #red; 
		
		//draw line([{0, 0}, {first(high_density_zone).wall_repulsion_velocity[grid_x][grid_y][0], 
		//					first(high_density_zone).wall_repulsion_velocity[grid_x][grid_y][1]}]) at: location color: #blue;
		
	}	
}

species center {
	aspect default {
		draw shape color: #yellow;
	}
}

species test_target {
	aspect default {
		draw shape color: #green;
	}
}

species mesh_centroid {
	aspect default {
		draw circle(0.1) at: shape.location color: #red border: #green;
	}
}

experiment exp parallel: true {
	output {
		//display my_display type: opengl {
		display my_display {
//			species center;
			species heat_map;
			species building;
//			species nav_mesh;		
			//species individual;
			//species high_density_zone;
//			species test_target;
		}
	}
}