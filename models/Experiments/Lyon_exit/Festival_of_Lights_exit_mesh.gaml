/**
* Name: FestivalofLightsexitmesh
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model FestivalofLightsexitmesh
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../Continuum_Crowds/high_density_zone.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'
/* Insert your model definition here */
global {
	//float step <- STEP;
	
	float step <- 0.1;
	
	file building_shape_file <- shape_file('../../includes/place_des_Terreaux/ok.shp');
	file road_shape_file <- shape_file('../../includes/place_des_Terreaux/lines.shp');
	file center_shape_file <- shape_file('../../includes/place_des_Terreaux/center2.shp');
	file init_space_shape_file <- shape_file('../../includes/place_des_Terreaux/init_space.shp');
	
	file nav_mesh_shape_file <- shape_file('../../includes/gen_map_official/mesh.shp');
	file mesh_centroid_shape_file <- shape_file('../../includes/gen_map_official/node.shp');
	
	geometry shape <- envelope(building_shape_file);
	geometry space <- copy(shape);
	point group_target <- {60, 60};
	
	// ------------
	float LENGTH_X;
	float LENGTH_Y;
	
	int nb_agents <- 6000;
	
	
	init {
		MODEL_NAME <- 'FestivalofLightsexitmesh';
		//MAXIMUM_SPEED <- 1.0 #m/#s;
		//MINIMUM_SPEED <- 0.1 #m/#s;
		MAXIMUM_SPEED <- 1.4 #m/#s;
		MINIMUM_SPEED <- 0.05 #m/#s;
		
		// ENVIRONMENTS
		create building from: building_shape_file;
		create center from: center_shape_file;
		
		create nav_mesh from: nav_mesh_shape_file;
		create mesh_centroid from: mesh_centroid_shape_file;

		loop c over: mesh_centroid {
			loop my_mesh over: nav_mesh {
				if c.shape overlaps	my_mesh.shape {
					my_mesh.centroid <- c.shape.location;
				}
			}	
		}
		
		loop mesh_i over: nav_mesh {
			loop mesh_j over: nav_mesh {
				if (mesh_j != mesh_i) {
					if mesh_i.shape intersects mesh_j.shape {
						mesh_i.neighbors <- mesh_i.neighbors + [mesh_j];
					}
				}
			}
		}
		
		// HIGH-DENSITY ZONE
		create high_density_zone {
			//GRID_WIDTH <- 30;
			//GRID_HEIGHT <- 30;
			
			GRID_WIDTH <- 60;
			GRID_HEIGHT <- 60;
			
			nb_groups <- 1;
			boundary <- [15.5, 75.0, 32.0, 70.5];
			float x_min <- boundary[0];
			float x_max <- boundary[1];
			float y_min <- boundary[2];
			float y_max <- boundary[3];
			
			CELL_SIZE_X <- (x_max - x_min) / GRID_WIDTH;
			CELL_SIZE_Y <- (y_max - y_min) / GRID_HEIGHT;
			
			CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			//group_goals <- [1::[[0, 29]]];
			group_goals <- [1::[[0, 59]]];
			
			do init_state;
			exit_meshes <- [1::nav_mesh(30)];
		}

		// INDIVIDUAL
		// operational level
		create avoiding_target number: 1;
//		create avoiding number: 1;
		create following number: 1;
		
		// tatical level
		create best_neighbor_mesh number: 1;
		
		// -----------------------------
//		GET_NEIGHBOR <- false;
		GET_NEIGHBOR <- true;
		
		LENGTH_X <- first(heat_map).shape.points[2].x;
		LENGTH_Y <- first(heat_map).shape.points[1].y;
		
		//write LENGTH_X;
		//write LENGTH_Y;
		
		// distance check for local target
		DISTANCE_CHECK <- 2.0;

		loop x from: 15 to: 23 {
			float y <- 0.92 * x + 50.56;
			create test_target {
				shape <- circle(0.2) at_location {x,y};
			}
		}
		
		create individual number: nb_agents {
			color <- #blue;
			target <- {17.68, 131.12};
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			
			group_id <- 1;
			
//			my_operational_behavior <- first(avoiding);
//			my_operational_behavior <- first(avoiding_target);
			my_operational_behavior <- first(following);
			
			my_tactical_behavior <- first(best_neighbor_mesh);
			
			location <- any_location_in(first(center));
			
			is_at_dense_region <- true;
			color <- #red;
			my_zone <- first(high_density_zone);
		}
		
	}
	
	//when: cycle > 10 and every(100#cycle)
	reflex detect_high_density_zone when: false {
		list<list<float>> my_data;
		loop ag over: individual {
			list<float> ag_coordinate <- [ag.location.x, ag.location.y];
			my_data <- my_data + [ag_coordinate];
		}

		list<float> bound <- detect_dense_region(my_data, 1.2, 8, 100, 2.0);
		
		if length(bound) > 0 {

			int x_min <- int(bound[0]);
			int x_max <- int(bound[1]);
			int y_min <- int(bound[2]);
			int y_max <- int(bound[3]);
		}
	}
	
//	reflex write_status when: every(100#cycle) {
//		write length(individual);
//	}
	
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
	
//	reflex save_velocities when: every(int(1/STEP) #cycle) {
//		ask individual {
//			save [velocity.x, velocity.y, is_at_dense_region] type: csv rewrite: false to: 'velocities.csv';
//		}
//	}

	reflex measure_simulation_time {
		if cycle = 2 {
			write #now;
		}
		
		if cycle = 102 {
			write #now;
		}
	}
	
	reflex count_and_save_exit_pedestrian when: false {
		write nb_exit_pedestrians;
		save [cycle, nb_exit_pedestrians] type: csv to: "data/outflow_ourmodel/sim5_" + string(nb_agents) + ".txt" rewrite: false;
	}
	
	reflex write_evacuation_time when: length(individual) = 0 {
		write("Evacuation Time: " + cycle * step);
		do pause;
	}
	
	reflex do_pause when: cycle in [600, 1500, 2400] {
		do pause;
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

//grid heat_map width: 130 height: 130 {
grid heat_map width: 80 height: 80 {
	float speed_sum;
	float average_speed;
	float density;
	int nb_agents;
	rgb my_color;
	
	action define_color {
		
		if density < 0.5 {
			float t <- density / 0.5;
			my_color <- rgb(0, 200 + (255 - 200) * t, 255);
			//my_color <- rgb(0, int(255 * t), 255);
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
		
		//draw line([{0, 0}, {first(high_density_zone).group_gradient_field_WE_SN[1][grid_x][grid_y][0], 
		//					first(high_density_zone).group_gradient_field_WE_SN[1][grid_x][grid_y][1]}]) at: location color: #red; 
		
//		draw line([{0, 0}, {average_velocity_field[grid_x][grid_y][0], 
//							average_velocity_field[grid_x][grid_y][1]}]) at: location color: #green;
		
	}	
}

experiment exp parallel: true {
	output {
		display my_display type: opengl {
		//display my_display {
//			species center;
//			species heat_map;
			species building;
//			species nav_mesh;		
			species individual;
			//species high_density_zone;
//			species test_target;
		}
	}
}

