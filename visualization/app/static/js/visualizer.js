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
        // SOURCE
        new ROS3D.PointCloud2({   // Point Cloud
            ros: ros,
            tfClient: tfClient,
            rootObject: viewerSource.scene,
            topic: '/registration_api/source/point_cloud',
            material: { size: 0.5, color: 0x940b01 },
            max_pts: 100000
        });
        new ROS3D.MarkerClient({  // Bounding Box
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/source/bounding_box',
            lifetime : 0,
            rootObject : viewerSource.scene
        });
        new ROS3D.MarkerArrayClient({  // Base B
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/base_b',
            lifetime : 0,
            rootObject : viewerSource.scene
        });
        // DESTINATION
        new ROS3D.PointCloud2({  // Point Cloud
            ros: ros,
            tfClient: tfClient,
            rootObject: viewerDestination.scene,
            topic: '/registration_api/destination/point_cloud',
            material: { size: 0.5, color: 0x000000 },
            max_pts: 100000
        });
        new ROS3D.MarkerClient({  // Bounding Box
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/destination/bounding_box',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });
        new ROS3D.MarkerArrayClient({   // Base U
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/base_u',
            lifetime : 0,
            rootObject : viewerDestination.scene
        });
        // TARGET
        new ROS3D.PointCloud2({   // Destination Point Cloud
            ros: ros,
            tfClient: tfClient,
            rootObject: viewer.scene,
            topic: '/registration_api/destination/point_cloud',
            material: { size: 0.3, color: 0x000000 },
            max_pts: 1000000
        });
        new ROS3D.MarkerArrayClient({  // Destination Base
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/base_u',
            lifetime : 0,
            rootObject : viewer.scene
        });
        new ROS3D.PointCloud2({   // Source Point Cloud Transformed
            ros: ros,
            tfClient: tfClient,
            rootObject: viewer.scene,
            topic: '/registration_api/source/point_cloud/transformed',
            material: { size: 0.3, color: 0x940b01 },
            max_pts: 1000000
        });
        new ROS3D.MarkerArrayClient({    // Source Base Transformed
            ros : ros,
            tfClient : tfClient,
            topic : '/registration_api/base_b/transformed',
            lifetime : 0,
            rootObject : viewer.scene
        });

        // Service Publish Point Cloud
        var destinationPublish = new ROSLIB.Service({
            ros: ros,
            name: '/registration_api/destination/publish',
            serviceType: 'std_srvs/Trigger'
        });

        // Service Publish Mesh
        var sourcePublish = new ROSLIB.Service({
            ros: ros,
            name: '/registration_api/source/publish',
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
