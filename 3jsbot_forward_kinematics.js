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
	robot.links[robot.base].xform = generate_identity(4);
	var threejs_xform = matrix_2Darray_to_threejs(robot.links[robot.base].xform);
	simpleApplyMatrix(robot.links[robot.base].geom,threejs_xform);
	for( var i = 0; i <  robot.links[robot.base].children.length; i++ )
		traverse_forward_kinematics_joint(robot.links[robot.base].children[i], generate_identity(4));
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
	robot.joints[joint].xform = matrix_multiply( translation, generate_rotation_matrix( robot.joints[joint].origin["rpy"][0], robot.joints[joint].origin["rpy"][1], robot.joints[joint].origin["rpy"][2]));
	robot.joints[joint].origin.xform = robot.joints[joint].xform;
	var threejs_xform = matrix_2Darray_to_threejs(robot.joints[joint].xform);
	simpleApplyMatrix(robot.joints[joint].geom,threejs_xform);
	if( robot.joints[joint].child != undefined )
		traverse_forward_kinematics_link( robot.joints[joint].child, robot.joints[joint].xform );
	return;
}

