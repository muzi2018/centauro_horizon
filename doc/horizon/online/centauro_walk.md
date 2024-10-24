roslaunch kyon_controller centauro_controller.launch

package: kyon_controller
executable1: controller src/controller_node.cpp; src/controller.cpp
config: centauro_controller_config.yaml

```xml
    <node pkg="kyon_controller" type="controller" name="centauro_controller_node" output="screen">
        <param name="config" textfile="$(find kyon_controller)/config/centauro_controller_config.yaml"/>
        <param name="rate" value="700"/>
    </node>
```

package: kyon_controller
executable2: centauro_receding_horizon.py

```xml
    <node pkg="kyon_controller" type="centauro_receding_horizon.py" name="mpc_node" output="screen">
        <param name="xbot" value="$(arg xbot)"/>
        <param name="joy" value="$(arg joy)"/>
        <rosparam param="closed_loop">False</rosparam>
    </node>

```

# executable1

## config

centauro_controller_config.yaml

MPC configure & Joint control model & Joint property

## resource(init class)

class Controller

```cpp
_joint_state_pub = _nh.advertise<xbot_msgs::JointState>("joint_state", 10);
```

class MPCJointHandler

```cpp
// subscribe pelvis's pose and twist
    _gt_pose_sub = _nh.subscribe("/xbotcore/link_state/pelvis/pose", 1, &MPCHandler::gt_pose_callback, this);
    _gt_twist_sub = _nh.subscribe("/xbotcore/link_state/pelvis/twist", 1, &MPCHandler::gt_twist_callback, this);

// subscribe mpc solution
_mpc_sub = _nh.subscribe("/mpc_solution", 1, &MPCJointHandler::mpc_joint_callback, this);
// publish the solution position resampled from mpc solution 
_resampler_pub = _nh.advertise<sensor_msgs::JointState>("/resampler_solution_position", 1, true);
```

## execution

```cpp
void Controller::run()
{

    if(_mpc_handler->isMsgReceived())
    {
        if (_init)
        {
            _mpc_handler->update();
            return;
        }
    }
    else
    {
        return;
    }

    if (!_init)
    {
        _init = true;
        // set_stiffness_damping_torque(0.01);

        if (!_stiffness_map.empty() || !_damping_map.empty())
        {
            set_stiffness_damping(1.);
        }

        _robot->setControlMode(_init_ctrl_map);
    }
}
```

# executable2

## config

centauro_config.yaml

Setting constraints & costs

## resource

```python
solution_publisher = rospy.Publisher('/mpc_solution', WBTrajectory, queue_size=1, tcp_nodelay=True)
```

```python
solution_time_publisher = rospy.Publisher('/mpc_solution_time', Float64, queue_size=1, tcp_nodelay=True)
```

## execution

firstly using bootstrap get a initial data

```python
ti.bootstrap()
# exit()
ti.load_initial_guess()
```

secondly using the initial data to warmstart the real-time mpc

```python
  while not rospy.is_shutdown():
    ti.rti()
    solution = ti.solution
```



defbootstrap(self):
	t=time.time()
	self.solver_bs.solve()


defrti(self):
	t=time.time()
	check=self.solver_rti.solve()



bootstrapping solver configurations...
{'ipopt.linear_solver': 'ma57', 'ipopt.tol': 0.1, 'ipopt.constr_viol_tol': 0.01, 'ilqr.constraint_violation_threshold': 0.01, 'ipopt.print_level': 5, 'ipopt.suppress_all_output': 'yes', 'ipopt.sb': 'yes', 'ilqr.suppress_all_output': 'yes', 'ilqr.codegen_enabled': True, 'ilqr.codegen_workdir': '/tmp/tyhio', 'ilqr.enable_gn': True, 'ilqr.hxx_reg_base': 0.0, 'ilqr.n_threads': 0, 'print_time': 0}


realtime_iter solver configurations...
{'ipopt.linear_solver': 'ma57', 'ipopt.tol': 0.1, 'ipopt.constr_viol_tol': 0.01, 'ilqr.constraint_violation_threshold': 0.01, 'ipopt.print_level': 5, 'ipopt.suppress_all_output': 'yes', 'ipopt.sb': 'yes', 'ilqr.suppress_all_output': 'yes', 'ilqr.codegen_enabled': True, 'ilqr.codegen_workdir': '/tmp/tyhio', 'ilqr.enable_gn': True, 'ilqr.hxx_reg_base': 0.0, 'ilqr.n_threads': 0, 'print_time': 0, 'ilqr.enable_line_search': False, 'ilqr.max_iter': 1}
