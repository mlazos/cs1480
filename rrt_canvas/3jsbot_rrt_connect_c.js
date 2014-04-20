//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/


function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here
    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();


    }

    // return path not currently found
    return false;
}

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
	tree.vertices[0].parent = -1;
    // create rendering geometry for base location of vertex configuration
    //add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;
    return tree;
}


/*function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}*/

//Custom functions

/*
CS148: reference code has functions for:

    tree_add_vertex  tree.vertices.push(vertex)
    tree_add_edge	tree.edges.push(edge)
    random_config 
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/

function tree_add_vertex(tree, parent_index, q )
{
	tree.newest = tree.newest + 1;
	tree.vertices[tree.newest] = {};
	tree.vertices[tree.newest].vertex = q;
	tree.vertices[tree.newest].parent = parent_index;
	tree.vertices[parent_index].edges.push(tree.newest);
	tree.vertices[tree.newest].edges = [];
	draw_2D_configuration(q);
	draw_2D_edge_configurations(q, tree.vertices[parent_index].vertex);
}

function rrt_connect_iterate()
{
	if( !finished )
	{
		var q_rand = random_config();
		console.log("random");
		console.log(q_rand);
		if(rrt_extend(treeA, q_rand))
		{
			var q_new = treeA.vertices[treeA.newest].vertex;
			if( rrt_connect(treeB, q_new) )
			{
				//finished
				finished = 1;
				path = generate_path(treeA.vertices[treeA.newest], treeB.vertices[treeB.newest]);		
				draw_highlighted_path(path);
			}
		}

		//swap treeA and treeB	
		var temp = treeA;
		treeA = treeB;
		treeB = temp;
	}
}


function rrt_connect(tree, q )
{
	var stop = 2;
	for( var i = 0; i < 50 && stop == 2; i++ )
	{
		var start = Date.now();
		while( Date.now() - start < 10 ){}
		stop = rrt_extend(tree, q);
	}
	return stop; //is 1 or 0 indicating reached or trapped
}

function rrt_extend(tree, q)
{
	var q_near_index = nearest_neighbor(q, tree);
	var q_near = tree.vertices[q_near_index].vertex;
	var q_new = new_config(q, q_near);
	
	if( !collision_test(q_new) )
	{
		tree_add_vertex(tree, q_near_index, q_new);
		if( config_equals(q_new, q) )
		{
			return 1;  // 1 for Reached
		}
		else
		{
			return 2; // 2 for advanced
		}
	}
	else
	{
		return 0; // 0 for "Trapped"
	}
	
}


function config_equals( q1, q2 )
{
	var cond = (q1[0] === q2[0]) && (q1[1] === q2[1]);
	console.log(cond);
	return cond;
}

function new_config(q, q_near)
{
	epsilon = .1;
	if( norm2d(q, q_near) < epsilon )
	{
		return q;
	}
	else
	{
		var delta_x = q[0] - q_near[0];
		var delta_y = q[1] - q_near[1];
		var angle = determine_angle(delta_x, delta_y);
		return [q_near[0] + Math.cos(angle) * epsilon, q_near[1] + Math.sin(angle) * epsilon, q[2], q[3], q[4], q[5] ];	
	}
}

function norm2d( q1, q2 )
{
	return Math.sqrt( (q1[0]-q2[0])*(q1[0]-q2[0]) + (q1[1]-q2[1])*(q1[1]-q2[1]) );
}

function random_config()
{
	var config =  [Math.random()*6.2 - 1.1, Math.random()*6.2 - 1.1, Math.random(), Math.random(), Math.random(), Math.random()];
	console.log(config);
	return config;
}

function nearest_neighbor( q, tree  )
{
	var min_dist = Infinity;
	var min_vertex = 0;
	var dist = 0;
	for( var i = 0; i < tree.vertices.length; i++ )
	{
		dist = norm2d( q, tree.vertices[i].vertex );
		if( dist < min_dist  )
		{
			min_vertex = i;
			min_dist = dist;
		}
	}
	
	return min_vertex;
}

function determine_angle( delta_x, delta_y )
{
	var val = delta_y/delta_x;

	if( delta_x < 0 && delta_y < 0)
		return Math.atan(val) + Math.PI;

	if( delta_x > 0 && delta_y < 0)
		return Math.atan(val);


	if( delta_x < 0 && delta_y > 0)
		return Math.atan(val) + Math.PI;
		
	if( delta_x > 0 && delta_y > 0);
		return Math.atan(val);

}

function generate_path( vA, vB )
{
	pathA = [];
	pathB = [];
	while( vA.parent != -1 )
	{
		pathA.push(vA);
		vA = treeA.vertices[vA.parent];
	}
	
	while( vB.parent != -1 )
	{
		pathB.push(vB);
		vB = treeB.vertices[vB.parent];
	}
	pathA.reverse();
	return pathA.concat( pathB );

}

