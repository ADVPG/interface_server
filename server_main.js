const express = require("express");
const { createServer } = require("http");
const { Server } = require("socket.io");

const app = express();
const httpServer = createServer(app);
const io = new Server(httpServer, { /* options */ });

const rosnodejs = require('rosnodejs');

const Jimp = require('jimp');

//var os = require("os"); // Solo si se usa localhost
//const hostname = os.hostname();
const hostname = '192.168.41.111';
//const hostname = '192.168.0.30';
const port = 3000;

//var get_acc = 'NoData'

// Wait for rosnodejs to init the ROS node
rosnodejs.initNode('rosnodejs_interface_node', { onTheFly: true })
.then((nh) => {

  // Load required ROS packages
  //rosnodejs.loadAllPackages();
  const std_msgs = rosnodejs.require('std_msgs');
  const geometry_msgs = rosnodejs.require('geometry_msgs');
  const ackermann_msgs = rosnodejs.require('ackermann_msgs');
  const pointcloud_to_image = rosnodejs.require('pointcloud_to_image');
  const conversion_coord_odom = rosnodejs.require('conversion_coord_odom');

  // Define services clients
  const coord_to_odom_client = nh.serviceClient('/coord_to_odom', 'conversion_coord_odom/CoordToOdom');
  const odom_to_coord_client = nh.serviceClient('/odom_to_coord', 'conversion_coord_odom/OdomToCoord');

  // Define publishers
  const navGoal_pub = nh.advertise('/move_base_simple/goal', 'geometry_msgs/PoseStamped');
  const navStart_pub = nh.advertise('/initialpose', 'geometry_msgs/PoseWithCovarianceStamped');
  const movControl_pub = nh.advertise('/desired_ackerman_state', 'ackermann_msgs/AckermannDriveStamped');
  const pcConfig_pub = nh.advertise('/pointCloud_config', 'pointcloud_to_image/pointcloud_to_img_config');
  /*
  // Create global messages
  const position_msg = new geometry_msgs.msg.PoseWithCovarianceStamped();
  let gnss_lat, gnss_lon;

  //console.log('Esperando...');
  //setTimeout(() => {

    // Pruebas

    //pcConfig_pub.publish({radial:20.3, polar:30.4, azimuth:15.3});

  //}, 5000);

  // Define suscribers and...
  // Suscribe to ROS topics and emit data extracted form messages by socket events
  
  // Get robot GNSS position
  const sub_gnss = nh.subscribe('/multiGNSS_Loc', 'sensor_msgs/NavSatFix', (msg) => {

    // Save GNSS to use in conversions between coord and odom 
    gnss_lat = msg.latitude;
    gnss_lon = msg.longitude;

  });

  // Get robot accurate position
  const sub_acc = nh.subscribe('/odom', 'nav_msgs/Odometry', (msg) => {
    
    //console.log('Got odometry: %j', msg);

    // Save position to start navigation
    position_msg.header = msg.header;
    position_msg.pose = msg.pose;
    
    nh.waitForService('/odom_to_coord', 1000)
      .then((available) => {

      if (available) {

        odom_to_coord_client.call({x: msg.pose.pose.position.x, y: msg.pose.pose.position.y, gnss_lat: gnss_lat, gnss_lon: gnss_lon})
        .then((resp) => { 
          //console.log('Lat: %d\tLon: %d', resp.lat, resp.lon);
          io.volatile.emit('rob:pose', {lat: resp.lat, lon: resp.lon}); //io.volatile.emit ??????
        })
        .catch(err => console.error(err));

      }
      else{console.log('Failed to call service odom_to_coord');}
      
    });
    
  });
  
  //Send postition any given time
  setInterval(() => {

    nh.waitForService('/odom_to_coord', 1000)
      .then((available) => {

      if (available) {

        odom_to_coord_client.call({x: position_msg.pose.pose.position.x, y: position_msg.pose.pose.position.y, gnss_lat: gnss_lat, gnss_lon: gnss_lon})
        .then((resp) => { 
          //console.log('Lat: %d\tLon: %d', resp.lat, resp.lon);
          io.volatile.emit('rob:pose', {lat: resp.lat, lon: resp.lon}); //io.volatile.emit ??????
        })
        .catch(err => console.error(err));

      }
      else{console.log('Failed to call service odom_to_coord');}
      
    });

  }, 5000);
  */
  
  // Get camera1 RGB image
  const sub_cam_RGB1 = nh.subscribe('/camera1/color/image_raw', 'sensor_msgs/Image', (msg) => {
    //console.log('Got msg: %j', msg);
    //io.emit('cam:RGB1', {width: msg.width, height: msg.height, data: msg.data});
    /*
    new Jimp({
      width: msg.width,
      height: msg.height,
      data: Buffer.from(msg.data)
    }, (err, image) => {
      v
    });
    */
    
    //console.log(msg.data.length)

    new Jimp({
      width: msg.width,
      height: msg.height,
      data: Buffer.from(msg.data)
    }, (err, image) => {
      image.write('output.png');
      //image.resize(50, Jimp.AUTO)
      image.write('output2.png');
      image.getBase64Async(Jimp.AUTO)
      .then(base64 => {
        console.log(base64);
        io.volatile.emit('cam:RGB1', base64);
      });
    });
  

  //var randomColor = '#' + Math.floor(Math.random()*16777215).toString(16);
  //console.log(randomColor);

  //io.volatile.emit('cam:RGB1', randomColor);
    
  });
  /*

  setInterval(() => {

    var randomColor = '#' + Math.floor(Math.random()*16777215).toString(16);
    console.log(randomColor);

    io.volatile.emit('cam:RGB1', randomColor);

  }, 2000);
  */
  /*
  // Get camera1 depth image
  const sub_cam_deepth1 = nh.subscribe('/camera1/aligned_depth_to_color/image_raw', 'sensor_msgs/Image', (msg) => {
    //console.log('Got msg: %j', msg);
    io.emit('cam:deepth1', msg.data);
  });

  // Get camera2 RGB image
  const sub_cam_RGB2 = nh.subscribe('/camera2/color/image_raw', 'sensor_msgs/Image', (msg) => {
    //console.log('Got msg: %j', msg);
    io.emit('cam:RGB2', msg.data);
  });

  // Get camera2 depth image
  const sub_cam_deepth2 = nh.subscribe('/camera2/aligned_depth_to_color/image_raw', 'sensor_msgs/Image', (msg) => {
    //console.log('Got msg: %j', msg);
    io.emit('cam:deepth2', msg.data);
  });

  // Get pointCloud image
  const sub_cam_pc = nh.subscribe('/pointCloud_image', 'sensor_msgs/Image', (msg) => {
    //console.log('Got msg: %j', msg);
    io.emit('cam:pc', msg.data);
  });
  */
  
  // Enable client conection to the server (and define callback function)
  io.on("connection", (socket) => {
  
    console.log('new connect', socket.id);
    /*
    // Listen for socket events and publish messages with recived data on ROS topics
    
    // Set navigation goals
    socket.on("map:marker", (arg) => {
      
      console.log('Recived navigation goal: ', arg);
      
      arg.forEach(function(element) {
        console.log(element);
      
        nh.waitForService('/coord_to_odom', 1000)
        .then((available) => {

          if (available) {

            coord_to_odom_client.call({lat: element[0], lon: element[1], gnss_lat: gnss_lat, gnss_lon: gnss_lon})
            .then((resp) => { 
              const msg = new geometry_msgs.msg.PoseStamped();
              msg.pose.position.x = resp.x;
              msg.pose.position.y = resp.y;
              navGoal_pub.publish(msg);
            })
            .catch(err => console.error(err));

          }
          else{console.log('Failed to call service coord_to_odom');}

        });

      });
      // Start navigation
      navStart_pub.publish(position_msg);
      

    });
    
    // Movement control
    socket.on("mov:control", (arg) => {
      
      console.log('Recived movement control: ', arg);

      const msg = new ackermann_msgs.msg.AckermannDriveStamped();
      msg.drive.speed = arg.velocity;
      msg.drive.steering_angle = arg.angle;

      movControl_pub.publish(msg);

    });

    // Arm control
    socket.on("arm", (arg) => {

      console.log('Arm: ', arg);

    });

    // Pinza control
    socket.on("arm:pinza", (arg) => {

      console.log('Pinza: ', arg);
      
    });

    // Set pointCloud configuration
    socket.on("lidar", (arg) => {
      
      console.log('Recived pointCLoud configuration: ', arg);

      pcConfig_pub.publish({height: 1080, width: 1080, radial: arg[2], polar: arg[0]*Math.PI/180, azimuth: arg[1]*Math.PI/180});

      //pcConfig_pub.publish({radial:arg.rad, polar:arg.pol, azimuth:arg.azm});

      //pcConfig_pub.publish(position_msg);

    });
  */
  });
  

});

/*
app.get('/acceleration', (req, res) => {
    res.send({acc:get_acc})
})
*/

httpServer.listen(port, hostname, () => {
    console.log(`Server running at http://${hostname}:${port}/`);
});