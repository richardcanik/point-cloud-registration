/**
* Setup all visualization elements when the page is loaded.
*/
function init() {
    console.log(destination)

    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
        console.log('Connection made!');

        // First, we create a Param object with the name of the param.
        var destinationPointCloud = new ROSLIB.Param({
            ros : ros,
            name : 'destination_point_cloud'
        });
        destinationPointCloud.set(destination);

    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'viewer',
      width : 1500,
      height : 800,
      antialias : true,
      background : '#f7f7f7',
    });
    viewer.camera.position.x = 400
    viewer.camera.position.y = 400
    viewer.camera.position.z = 400

    // Add a grid.
    var grid = new ROS3D.Grid();
    grid.scale.x = 100;
    grid.scale.y = 100;
    grid.scale.z = 100;
    viewer.addObject(grid);

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/map'
    });

    var cloudClient = new ROS3D.PointCloud2({
        ros: ros,
        tfClient: tfClient,
        rootObject: viewer.scene,
        topic: '/destination_point_cloud',
        material: { size: 1, color: 0xff00ff },
        max_pts: 10000000
    });

}