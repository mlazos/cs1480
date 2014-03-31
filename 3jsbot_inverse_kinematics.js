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

funtction robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
        //iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        for( link in robot.links )
		{
			if( link.children == [] )
				var end_xform = link.xform;
		}					

		endeffector_geom.visible = true;
        target_geom.visible = true;
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}
