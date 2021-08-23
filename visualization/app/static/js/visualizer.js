$( document ).ready(function() {
    // Source Viewer
    var viewerSource = new ROS3D.Viewer({
        divID : 'source-viewer',
        width : $('#source-viewer').width() * 0.98,
        height : 500,
        antialias: true,
        background : '#f7f7f7',
    });
    viewerSource.camera.position.x = 100;
    viewerSource.camera.position.y = 100;
    viewerSource.camera.position.z = 100;
    viewerSource.camera.updateMatrixWorld();

    // Destination Viewer
    var viewerDestination = new ROS3D.Viewer({
        divID : 'destination-viewer',
        width : $('#destination-viewer').width() * 0.98,
        height : 500,
        antialias: true,
        background : '#f7f7f7',
    });
    viewerDestination.camera.position.x = 400;
    viewerDestination.camera.position.y = 400;
    viewerDestination.camera.position.z = 400;
    viewerDestination.camera.updateMatrixWorld();

    // Viewer
    var viewer = new ROS3D.Viewer({
        divID : 'viewer',
        width : $('#viewer').width()* 0.985,
        height : 800,
        antialias: true,
        background : '#f7f7f7',
    });
    viewer.camera.position.x = 200;
    viewer.camera.position.y = 200;
    viewer.camera.position.z = 200;
    viewer.camera.updateMatrixWorld();

    // ROS Websocket
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
        console.log('Connected');

        // TF Client
        var tfClient = new ROSLIB.TFClient({
            ros : ros,
            angularThres : 0.01,
            transThres : 0.01,
            rate : 10.0,
            fixedFrame : '/map'
        });

        // Source BaseB
        var sourceBaseB = new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/localization/base_b',
            lifetime : 0,
            rootObject : viewerSource.scene
        });

        // Source Bounding Box
        var sourceBoundingBox = new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/localization/source/bounding_box',
            lifetime : 0,
            rootObject : viewerSource.scene
        });

        // Source Mesh
        var sourceMesh = new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/localization/source/mesh',
            lifetime : 0,
            rootObject : viewerSource.scene
        });

        // Point Cloud Source
        var sourcePointCloud = new ROS3D.PointCloud2({
            ros: ros,
            tfClient: tfClient,
            rootObject: viewerSource.scene,
            topic: '/localization/source/point_cloud',
            material: { size: 2, color: 0x000000 },
            max_pts: 1000000
        });

        // Destination Point Cloud
//        var destinationPointCloud = new ROS3D.PointCloud2({
//            ros: ros,
//            tfClient: tfClient,
//            rootObject: viewerDestination.scene,
//            topic: '/localization/destination/point_cloud',
//            material: { size: 1, color: 0x000000 },
//            max_pts: 1000000
//        });

        // Bounding Box Destination
        var destinationBoundingBox = new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/localization/destination/bounding_box',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        // Destination Debug
        var destinationDebug = new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/localization/debug',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        // Destination Sphere1
        new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/octo_map/points1',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        // Destination Sphere2
        new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/octo_map/points2',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/octo_map/points3',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        new ROS3D.MarkerClient({
            ros : ros,
            tfClient : tfClient,
            topic : '/octo_map/points4',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });

        // Service Publish Point Cloud
        var destinationPublish = new ROSLIB.Service({
            ros: ros,
            name: '/localization/destination/publish',
            serviceType: 'std_srvs/Trigger'
        });

        // Service Publish Mesh
        var sourcePublish = new ROSLIB.Service({
            ros: ros,
            name: '/localization/source/publish',
            serviceType: 'std_srvs/Trigger'
        });

        console.log('source: ', source)
        console.log('destination: ', destination)
        if(destination != 'None') {
            // Service Call Publish
            setTimeout(function() {
                destinationPublish.callService(new ROSLIB.ServiceRequest({}), function (response) {});
            }, 500);
        }

        if(source != 'None') {
            // Service Call Publish
            setTimeout(function() {
                sourcePublish.callService(new ROSLIB.ServiceRequest({}), function (response) {});
            }, 500);
        }
    });
});
