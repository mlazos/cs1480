//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/
function robot_forward_kinematics() // use mutual recursion to move through the kinematic hierarchy
{
	var translation = generate_translation_matrix( robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2] ); 
	var rotation = generate_rotation_matrix( 0, robot.origin.rpy[1], 0 );
	robot_lateral = matrix_multiply( rotation, generate_heading() );
	robot_heading = matrix_multiply(generate_rotation_matrix(0, -Math.PI/2, 0), robot_lateral);
	robot.links[robot.base].xform = matrix_multiply(translation, rotation ); //genenrate_identity(4);
	var threejs_xform = matrix_2Darray_to_threejs(robot.links[robot.base].xform);
	simpleApplyMatrix(robot.links[robot.base].geom,threejs_xform);
	for( var i = 0; i <  robot.links[robot.base].children.length; i++ )
		traverse_forward_kinematics_joint(robot.links[robot.base].children[i], robot.links[robot.base].xform);
	return;
}

function traverse_forward_kinematics_link( link, matrix_stack ) //set transform to the current stack, and recursively call on each child joint
{
	robot.links[link].xform = matrix_stack;
	var threejs_xform = matrix_2Darray_to_threejs(robot.links[link].xform);
	simpleApplyMatrix(robot.links[link].geom,threejs_xform);	
	for( var i = 0; i <  robot.links[link].children.length; i++ )
		traverse_forward_kinematics_joint(robot.links[link].children[i], matrix_stack);
	return;
}

function traverse_forward_kinematics_joint( joint, matrix_stack ) //generate tanslation and rotation matrices and right multiply with the matrix stack
{	
	var translation = matrix_multiply( matrix_stack,  generate_translation_matrix( robot.joints[joint].origin["xyz"][0], robot.joints[joint].origin["xyz"][1], robot.joints[joint].origin["xyz"][2]) );
	var translation_rotation = matrix_multiply( translation, generate_rotation_matrix( robot.joints[joint].origin["rpy"][0], robot.joints[joint].origin["rpy"][1], robot.joints[joint].origin["rpy"][2]));
	robot.joints[joint].xform = matrix_multiply( translation_rotation, quaternion_to_rotation_matrix( quaternion_from_axisangle( robot.joints[joint].angle, robot.joints[joint].axis ) )); 
	robot.joints[joint].origin.xform = translation_rotation;//robot.joints[joint].xform;
	var threejs_xform = matrix_2Darray_to_threejs(robot.joints[joint].xform);
	simpleApplyMatrix(robot.joints[joint].geom,threejs_xform);
	if( robot.joints[joint].child != undefined )
		traverse_forward_kinematics_link( robot.joints[joint].child, robot.joints[joint].xform );
	return;
}

function generate_heading()
{
	outmat = init_matrix(4,1);
	outmat[0][0] = 1;
	outmat[1][0] = 0; 
	outmat[2][0] = 0;
	outmat[3][0] = 1;
	return outmat;
}

