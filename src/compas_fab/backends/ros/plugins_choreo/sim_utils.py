from conrob_pybullet import joints_from_names, link_from_name, set_joint_positions, \
    wait_for_duration, wait_for_user, create_attachment

def display_picknplace_trajectories(robot, ik_joint_names, ee_link_name,
                                    unit_geos, element_seq, trajectories, \
                                    ee_attachs=[],
                                    cartesian_time_step=0.075, transition_time_step=0.1, step_sim=False):
    # enable_gravity()
    ik_joints = joints_from_names(robot, ik_joint_names)
    end_effector_link = link_from_name(robot, ee_link_name)

    for seq_id, unit_picknplace in enumerate(trajectories):
        handles = []
        unit_geo = unit_geos[element_seq[seq_id]]

        print('seq #{} : place 2 pick tranisiton'.format(seq_id))
        if 'place2pick' in unit_picknplace and unit_picknplace['place2pick']:
            # place2pick transition
            for conf in unit_picknplace['place2pick']:
                set_joint_positions(robot, ik_joints, conf)
                for ea in ee_attachs: ea.assign()
                wait_for_duration(transition_time_step)
        else:
            print('seq #{} does not have place to pick transition plan found!'.format(seq_id))

        if step_sim: wait_for_user()

        print('seq #{} : pick approach'.format(seq_id))
        # pick_approach
        for conf in unit_picknplace['pick_approach']:
            set_joint_positions(robot, ik_joints, conf)
            for ea in ee_attachs: ea.assign()
            wait_for_duration(cartesian_time_step)

        if step_sim: wait_for_user()

        # pick attach
        attachs = []
        for e_body in unit_geo.pybullet_bodies:
            attachs.append(create_attachment(robot, end_effector_link, e_body))
        # add_fixed_constraint(brick.body, robot, end_effector_link)

        print('seq #{} : pick retreat'.format(seq_id))
        # pick_retreat
        for conf in unit_picknplace['pick_retreat']:
            set_joint_positions(robot, ik_joints, conf)
            for ea in ee_attachs: ea.assign()
            for at in attachs: at.assign()
            wait_for_duration(cartesian_time_step)

        if step_sim: wait_for_user()

        print('seq #{} : pick 2 place tranisiton'.format(seq_id))
        # pick2place transition
        if 'pick2place' in unit_picknplace and unit_picknplace['pick2place']:
            for conf in unit_picknplace['pick2place']:
                set_joint_positions(robot, ik_joints, conf)
                for ea in ee_attachs: ea.assign()
                for at in attachs: at.assign()
                wait_for_duration(transition_time_step)
        else:
            print('seq #{} does not have pick to place transition plan found!'.format(seq_id))

        if step_sim: wait_for_user()

        print('seq #{} : place approach'.format(seq_id))
        # place_approach
        for conf in unit_picknplace['place_approach']:
            set_joint_positions(robot, ik_joints, conf)
            for ea in ee_attachs: ea.assign()
            for at in attachs: at.assign()
            wait_for_duration(cartesian_time_step)

        if step_sim: wait_for_user()

        # place detach
        # remove_fixed_constraint(brick.body, robot, end_effector_link)

        print('seq #{} : place retreat'.format(seq_id))
        # place_retreat
        for conf in unit_picknplace['place_retreat']:
            set_joint_positions(robot, ik_joints, conf)
            for ea in ee_attachs: ea.assign()
            wait_for_duration(cartesian_time_step)

        if step_sim: wait_for_user()
