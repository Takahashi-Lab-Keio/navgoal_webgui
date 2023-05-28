/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 * @modified by Fumiya Ohnishi - fumiya-onishi@keio.jp
 */

var NAV2D = NAV2D || {
  REVISION : '0.5.0-SNAPSHOT'
};

/**
 * USE INTERNALLY. Resize an Image map when receive new dimension.
 *
 * @param old_state - Previous state
 * @param viewer - Viewer 2D
 * @param currentGrid - Current grid with information about width, height and position
 */
NAV2D.resizeMap = function(old_state, viewer, currentGrid) {
  if(!old_state){
    old_state = {
      width: currentGrid.width,
      height: currentGrid.height,
      x: currentGrid.pose.position.x,
      y: currentGrid.pose.position.y
    };
    viewer.scaleToDimensions(currentGrid.width, currentGrid.height);
    viewer.shift(currentGrid.pose.position.x, currentGrid.pose.position.y);
  }
  if (old_state.width !== currentGrid.width || old_state.height !== currentGrid.height) {
    viewer.scaleToDimensions(currentGrid.width, currentGrid.height);
    old_state.width = currentGrid.width;
    old_state.height = currentGrid.height;
  }
  if (old_state.x !== currentGrid.pose.position.x || old_state.y !== currentGrid.pose.position.y) {
    viewer.shift((currentGrid.pose.position.x - old_state.x)/1, (currentGrid.pose.position.y - old_state.y)/1);
    old_state.x = currentGrid.pose.position.x;
    old_state.y = currentGrid.pose.position.y;
  }
  return old_state;
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 * @author Raffaello Bonghi - raffaello.bonghi@officinerobotiche.it
 * @modified by Fumiya Ohnishi - fumiya-onishi@keio.jp
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * use_tf (optional) - whther use tf or not
 *   * map_frame (optional) - map tf name
 *   * base_frame (optional) - robot tf name
 *   * robot_pose (optional) - the robot pose topic name
 *   * topicName (optional) - topic name of publishing PoseStamped msg
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: true)
 */
NAV2D.RobotPositionViewer = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var use_tf = options.use_tf && true;
  var map_frame = options.map_frame || '/map';
  var base_frame = options.base_frame || '/base_link';
  var robot_pose = options.robot_pose || '/robot_pose';
  var withOrientation = options.withOrientation && true;
  this.rootObject = options.rootObject || new createjs.Container();

  this.goalMarker = null;

  // setup tfClient
  var tfClient = new ROSLIB.TFClient({
    ros: ros,
    fixedFrame: map_frame,
    angularThres: 0.01,
    transThres: 0.01
  });

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = null;
  robotMarker = new ROS2D.NavigationArrow({
    size : 15,
    strokeSize : 1,
    fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
    pulse : false
  });

  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  var updateRobotPosition = function(pose, orientation) {
    // update the robots position on the map
    robotMarker.x = pose.x;
    robotMarker.y = -pose.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }
    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(orientation);
    // Set visible
    robotMarker.visible = true;
  };

  if(use_tf === true) {
    tfClient.subscribe(base_frame, function(tf) {
      updateRobotPosition(tf.translation,tf.rotation);
    });
  } else {
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros: ros,
      name: robot_pose,
      messageType: 'geometry_msgs/Pose',
      throttle_rate: 100
    });
    poseListener.subscribe(function(pose) {
      updateRobotPosition(pose.position,pose.orientation);
    });
  }
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @modified by Fumiya Ohnishi - fumiya-onishi@keio.jp
 */

/**
 * A RobotPositionViewerClient uses an OccupancyGridClient to create a map for use with a RobotPositionViewer.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * use_tf (optional) - whther use tf or not
 *   * map_frame (optional) - map tf name
 *   * base_frame (optional) - robot tf name
 *   * robot_pose (optional) - the robot pose topic name
 *   * rootObject (optional) - the root object to add this marker to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: true)
 *   * viewer - the main viewer to render to
 */
NAV2D.RobotPositionViewerClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var use_tf = options.use_tf && true;
  var map_frame = options.map_frame || '/map';
  var base_frame = options.base_frame || '/base_link';
  var map_topic = options.map_topic || '/map';
  var robot_pose = options.robot_pose || '/robot_pose';
  var continuous = options.continuous;
  var rootObject = options.rootObject || new createjs.Container();
  var viewer = options.viewer;
  var withOrientation = options.withOrientation && true;
  var old_state = null;

  // setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros : ros,
    rootObject : rootObject,
    continuous : continuous,
    topic : map_topic
  });

  var navigator = new NAV2D.RobotPositionViewer({
    ros: ros,
    use_tf: use_tf,
    map_frame: map_frame,
    base_frame: base_frame,
    robot_pose : robot_pose,
    rootObject: rootObject,
    withOrientation: withOrientation,
  });

  client.on('change', function() {
    // scale the viewer to fit the map
    old_state = NAV2D.resizeMap(old_state, viewer, client.currentGrid);
  });
};
