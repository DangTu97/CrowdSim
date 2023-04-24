/**
* Name: testmesh
* Based on the internal empty template. 
* Author: hdang
* Tags: 
*/


model bettermesh

/* Insert your model definition here */

global {
	shape_file mesh_shape_file <- shape_file('../../../includes/gen_map_official/mesh.shp');
	shape_file node_shape_file <- shape_file('../../../includes/gen_map_official/node.shp');
	geometry shape <- envelope(mesh_shape_file);
	
	init {
		create my_nav_mesh from: mesh_shape_file;
		create nav_centroid from: node_shape_file;
		
		loop mesh_i over: my_nav_mesh {
			loop mesh_j over: my_nav_mesh {
				if (mesh_j != mesh_i) {
					if mesh_i.shape intersects mesh_j.shape {
						mesh_i.neighbors <- mesh_i.neighbors + [mesh_j];
					}
				}
			}
		}
		
	}
}

species my_nav_mesh {
	point centroid;
	
	list<my_nav_mesh> neighbors;
	
	aspect default {
		draw shape color: #white border: #green;
	}
}

species nav_centroid {
	point center;
	aspect default {
		draw circle(0.1) color:#red at: center;
	}
}

experiment exp {
	output {
		display my_display {
			species my_nav_mesh;
			species nav_centroid;
		}
	}
}