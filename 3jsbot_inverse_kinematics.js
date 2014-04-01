//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {

		var link_xform = robot.joints[endeffector_joint].xform;
		var translation = generate_translation_matrix( endeffector_local_pos[0][0], endeffector_local_pos[1][0], endeffector_local_pos[2][0] );
		var end_xform = matrix_multiply( link_xform, translation );

		var target_translation = generate_translation_matrix( target_pos[0][0], target_pos[1][0], target_pos[2][0] );
        iterate_inverse_kinematics(target_translation, endeffector_joint, end_xform);
		
		var endeffector_mat = matrix_2Darray_to_threejs( end_xform );
		simpleApplyMatrix( endeffector_geom, endeffector_mat );
		endeffector_geom.visible = true;
        
		target_geom.visible = true;
		var target_mat = matrix_2Darray_to_threejs(  target_translation );
		simpleApplyMatrix( target_geom, target_mat );		
		
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}

function iterate_inverse_kinematics( target_xform, endeffector_joint, end_xform)
{
	var delta_x = [[target_xform[0][3] - end_xform[0][3]],
				   [target_xform[1][3] - end_xform[1][3]],
				   [target_xform[2][3] - end_xform[2][3]]];
	var alpha = 0.05;
	var joint_chain_start = [];
	var joint_chain = gen_kinematic_chain_joint( endeffector_joint, joint_chain_start );
	var jacobian = init_matrix( 3, joint_chain.length ); 	
	console.log(joint_chain);	
	On = new Array(3);
	On[0] = end_xform[0][3];
	On[1] = end_xform[1][3];
	On[2] = end_xform[2][3];
	
	for( var i = 0; i < joint_chain.length; i++ )
	{
		jacobian = fill_jacobian_col(i, jacobian, joint_chain[i], On);
	}

	console.log(jacobian);
	console.log(matrix_multiply( jacobian, matrix_transpose(jacobian) ) );
	var inverted_jacobian_transpose = numeric.inv( matrix_multiply( jacobian, matrix_transpose(jacobian) ) );
	var delta_theta = matrix_multiply( matrix_multiply(matrix_transpose(jacobian), inverted_jacobian_transpose), delta_x );	

	for( var i = 0; i < joint_chain.length; i++ )
	{
		robot.joints[joint_chain[i]].control = alpha * delta_theta[i][0];
	}
}

function gen_kinematic_chain_joint( joint, joint_chain  )
{
	joint_chain.push(joint);
	return gen_kinematic_chain_link( robot.joints[joint].parent, joint_chain );
}

function gen_kinematic_chain_link( link, joint_chain  )
{
	if( link == "base" )
	{
		return joint_chain;
	}
	else
	{ 
		return gen_kinematic_chain_joint( robot.links[link].parent, joint_chain );
	}
}

function fill_jacobian_col( col, jacobian, joint, On  )
{
	var joint_vector = new Array(3);
	joint_vector[0] = robot.joints[joint].xform[0][3];
	joint_vector[1] = robot.joints[joint].xform[1][3];
	joint_vector[2] = robot.joints[joint].xform[2][3];
	
	var diff_vector = new Array(3);
	diff_vector[0] = On[0] - joint_vector[0];
	diff_vector[1] = On[1] - joint_vector[1];
	diff_vector[2] = On[2] - joint_vector[2];

	var axis = robot.joints[joint].axis;

	var j_col = vector_cross( axis, diff_vector );

	jacobian[0][col] = j_col[0];
	jacobian[1][col] = j_col[1];
	jacobian[2][col] = j_col[2];

	return jacobian;
}



