# Task Type

horizon/rhc/taskInterface.py

```python
class TaskInterface(ProblemInterface):
    def __init__(self,
                 prb,
                 model):

        super().__init__(prb, model)

        # here I register the the default tasks
        # todo: should I do it here?
        # todo: name of task should be inherited from the task class itself:
        #  --> task_factory.register(CartesianTask.signature(), CartesianTask)
        # uniform the names of these tasks
        task_factory.register('Cartesian', CartesianTask)
        task_factory.register('Contact', ContactTask)
        task_factory.register('Wrench', SurfaceContact)
        task_factory.register('VertexForce', VertexContact)
        task_factory.register('Postural', PosturalTask)
        task_factory.register('JointLimits', JointLimitsTask)
        task_factory.register('Regularization', RegularizationTask)
        task_factory.register('Rolling', RollingTask)
        task_factory.register('Zmp', ZmpTask)
```

1. CartesianTask
2. ContactTask
3. SurfaceContact
4. VertexContact
5. PosturalTask
6. JointLimitsTask
7. RegularizationTask
8. RollingTask
9. ZmpTask

constraints:
	-contact_1
		type: Contact
		subtask: [interaction_1,rolling_task_1]
			interaction_1:
				type: VertexContact (limite force value of contact_1)
			rolling_task_1:
				type: Rolling (move the wheel)
    	-contact_2
	-contact_3
	-contact_4

costs:
	-joint_regularization
		type: Regularization (penalt the joints' vel and acc for stability)
	-joint_posture
		type:Postural (keeping initial position)
