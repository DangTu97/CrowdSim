/**
* Name: allSFM
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model allSFM
import '../../global_variables.gaml'
import '../../Individual/individual.gaml'
import '../../Continuum_Crowds/high_density_zone.gaml'
import '../../env.gaml'
import '../../helpers/detection.gaml'
/* Insert your model definition here */
global {
	float step <- STEP;
	
	file building_shape_file <- shape_file('../../includes/place_des_Terreaux/ok.shp');
	file road_shape_file <- shape_file('../../includes/place_des_Terreaux/lines.shp');
	file center_shape_file <- shape_file('../../includes/place_des_Terreaux/center_SFM_only.shp');
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
		MODEL_NAME <- "all_SFM";
		
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
			GRID_WIDTH <- 30;
			GRID_HEIGHT <- 30;
			nb_groups <- 1;
			boundary <- [100.0, 120.0, 100.0, 120.0];
			int x_min <- int(boundary[0]);
			int x_max <- int(boundary[1]);
			int y_min <- int(boundary[2]);
			int y_max <- int(boundary[3]);
			CELL_SIZE_X <- (x_max - x_min) / GRID_WIDTH;
			CELL_SIZE_Y <- (y_max - y_min) / GRID_HEIGHT;
			CELL_AREA <- CELL_SIZE_X * CELL_SIZE_Y;
			group_goals <- [1::[[0, 29]]];
			exit_meshes <- [1::nav_mesh(30)];
			do init_state;
		}

		// INDIVIDUAL
		// operational level
		create avoiding_SFM number: 1;
		create following number: 1;
		
		// tatical level
		create tactical_all_SFM number: 1;
		
		create point_target {
			shape <- {18.5, 90};
		}
		
		create point_target {
			shape <- {5, 67};
		}
		
		create individual number: nb_agents {
			local_target <- {15.76, 70.34};

			target <- one_of([{5, rnd(65.5, 69.5)}, {rnd(16.5, 21), 85}]);
					
			velocity <- (target - location) * rnd(0.3, MAXIMUM_SPEED)/ norm(target - location);
			group_id <- 1;
			
			my_operational_behavior <- first(avoiding_SFM);
			my_tactical_behavior <- first(tactical_all_SFM);
			
			location <- any_location_in(first(center));
			current_mesh <- nav_mesh closest_to self;
					
			is_at_dense_region <- false;
			color <- #blue;
			my_zone <- first(high_density_zone);
		}
		
		GET_NEIGHBOR <- true;
		
		LENGTH_X <- first(heat_map).shape.points[2].x;
		LENGTH_Y <- first(heat_map).shape.points[1].y;
		write LENGTH_X;
		write LENGTH_Y;
		
		// distance check for local target
		DISTANCE_CHECK <- 2.0;
	}
	
	//when: cycle > 10 and every(100#cycle)
	reflex detect_high_density_zone when: false {
		list<list<float>> my_data;
		loop ag over: individual {
			list<float> ag_coordinate <- [ag.location.x, ag.location.y];
			my_data <- my_data + [ag_coordinate];
		}

		list<float> bound <- detect_dense_region(my_data, 2.0, 10, 100, 2.0);
		
		if length(bound) > 0 {

			int x_min <- int(bound[0]);
			int x_max <- int(bound[1]);
			int y_min <- int(bound[2]);
			int y_max <- int(bound[3]);
			
			ask first(high_density_zone) {
				shape <- polyline([{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}, {x_min, y_min}]);
			}

		}
	}
	
	reflex draw_heat_map when: true {

		ask heat_map {
			nb_agents <- 0;
			speed_sum <- 0.0;
		}
		
		loop ag over: individual {
			int idx <- int(ag.location.x / LENGTH_X);
			int idy <- int(ag.location.y / LENGTH_Y);
			ask heat_map[idx, idy] {
				nb_agents <- nb_agents + 1;
				speed_sum <- speed_sum + norm(ag.velocity);
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
	
	reflex measure_simulation_time when: false {
		if cycle = 2 {
			write #now;
		}
		
		if cycle = 102 {
			write #now;
		}
	}
	
	reflex count_and_save_exit_pedestrian when: true {
		ask individual {
			if (- 0.92 * location.x + location.y > 50.56) and local_target = {15.76, 70.34} {
				do count_exit_pedestrians;
			}
		}
		
		write(nb_exit_pedestrians);
		save [cycle, nb_exit_pedestrians] type: csv to: "data/outflow_allSFM/sim2_" + string(nb_agents) + ".txt" rewrite: false;
	}
	
	reflex write_evacuation_time when: length(individual) = 0 {
		write("Evacuation Time: " + time * step);
		do pause;
	}
	
	reflex do_pause when: cycle in [600, 1500, 2400] {
		do pause;
	}
	
}

species point_target {
	aspect default {
		draw (shape + 0.2) color: #green;
	}
}

species center {
	aspect default {
		draw shape color: #yellow;
	}
}

species mesh_centroid {
	aspect default {
		draw circle(0.1) at: shape.location color: #red border: #green;
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
		
		float new_density <- nb_agents / (LENGTH_X * LENGTH_Y);
		//density <- (density + new_density) / 2;
		density <- new_density / 2;
		average_speed <- nb_agents = 0 ? 0.0 : speed_sum / nb_agents;
	}

	aspect default {
		draw shape color: my_color border: my_color wireframe: false;
	}	
	
}

experiment exp parallel: true  {
	
	init {
//		create simulation with:[sim_name::"sim2.txt"];
//		create simulation with:[sim_name::"sim3.txt"];
//		create simulation with:[sim_name::"sim4.txt"];
//		create simulation with:[sim_name::"sim5.txt"];
	}
	
	output {
		display my_display  {
			species heat_map;
			species building;
			species individual;
		}
	}
}
