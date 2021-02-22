# Gazebo Geometries Plugin
A world plugin to obtain collision geometries for an object in the Gazebo world. It uses a ROS service that takes in a model or collision link name and returns a box collision geometry for it. All geometries are obtained as boxes for simplicity's sake.

This plugin was developed while working on a 3rd Year Project at the University of Leeds for manipulation planning in cluttered environments.

If a model is requested, an array containing geometries for all its collision links is returned. If a collision link is requested, an array of size 1 is returned with only the geometry for that collision link.

The boxes are either Object Oriented Bounding Boxes (OOBB) or Axis Aligned Bounding Boxes (AABB). OOBBs are obtained for box and cylinder shaped objects and collision links, any other shape is treated as an AABB. It should be noted that the service returns enough information to allow the user to compute the AABB for the OOBB as well if needed.

To include this plugin in a Gazebo world, you will need to include it in the .world file inside the ```<world>``` tag.  
As an example:
```
<world>
    <plugin name='gazebo_geometries_plugin filename='libgazebo_geometries_plugin.so'/>
</world>
```

## Note on Kinova OOBBs

This package was developed for work on a project involving the Kinova Jaco2 manipulators, hence, support for OOBBs for those arms was hard coded in using collision link dimensions for those arms due to the URDFs using mesh shape types.

The package will still work as normal without the need to spawn the arms in Gazebo.

## Rviz Visualisation

The plugin publishes the geometries as visualisation markers on the topic ```/geometries_markers```.

There is a parameter, ```/gazebo/static_objs```, that can be set to visualise static objects (shown as red). It is an array of strings of model names that are regarded as static, these are not related to the ```static``` attribute within Gazebo. All geometries are displayed green unless they are specified as static.

Another parameter, ```/gazebo/pub_arm_geom```, can be set to visualise the Kinova collision boxes on the same topic as the other geometries.

These parameters are present in the [gazebo_params](../kinova_gazebo/params/gazebo_params.yaml) file which is in the [kinova_gazebo](../kinova_gazebo) folder.

## The Geometry Service

The service is advertised on ```/gazebo/get_geometry``` and takes in a string parameter which is the model or collision link name.

To call it through the command line, you can use:

```
$ rosservice call /gaezbo/get_geometry "model_name: 'name'" 
```

Replace ```name``` with the name of an object or collision link in the Gazebo world.

the returned variables are as follows:

- ```bool success``` - false if plugin is unable to find the requested model.
- ```string message``` - status message, it was added mainly if the service is being used through a command line.
- ```string[] name``` - the link name that the collision box corresponds to.
- ```geometry_msgs/Vector3[] min_bounds``` - minimum bounds of the box in X, Y, & Z. This essentially is the AABB minimum coordinates.
- ```geometry_msgs/Vector3[] max_bounds``` - maximum bounds of the box in X, Y, & Z. This essentially is the AABB maximum coordinates.
- ```geometry_msgs/Vector3[] dimensions``` - dimensions of the box in X, Y, & Z. In the case of the AABB, this will be the difference between the min and max bounds. For the OOBB, these will correspond to the dimensions of the object when its rotation is aligned with the world axes (i.e. identity), hence the rotation from the pose will need to be applied to obtain the rotated object's dimensions along each axes.
- ```geometry_msgs/Pose[] pose``` - pose of the box. The quaterion for the AABB will always be an identity (by definition of AABB).
