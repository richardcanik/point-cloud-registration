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

        var destinationSet = new ROSLIB.Service({
            ros: ros,
            name: '/destination/set',
            serviceType: 'localization_msgs/String'
        });

        var destinationPublish = new ROSLIB.Service({
            ros: ros,
            name: '/destination/publish',
            serviceType: 'std_srvs/Trigger'
        });

        if(destination != 'None') {
            console.log('Point Cloud Requested');

            destinationSet.callService(
            new ROSLIB.ServiceRequest({
                data: destination
            }),
            function (response) {
                console.log('Result for service call power_state = ' + response);
            });

            destinationPublish.callService(
            new ROSLIB.ServiceRequest({}),
            function (response) {
                console.log('Result for service call power_state = ' + response);
            });
        }
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
        divID : 'viewer',
        width : 1500,
        height : 800,
        background : '#f7f7f7',
    });
    viewer.camera.position.x = 200
    viewer.camera.position.y = 200
    viewer.camera.position.z = 200

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
        topic: '/destination',
        material: { size: 2, color: 0x000000 },
        max_pts: 10000000
    });

}