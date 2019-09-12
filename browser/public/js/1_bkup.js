'use strict'

const module_num = 1;                     // Module number

/*********************
 *   HTML Elements   *
 *********************/
var bar_ctx = [];
for(let j=0; j<JOINTS; j++) {
    bar_ctx.push(document.getElementById('bar' + (j+1)).getContext('2d'));
}

/*******************************
 *   Setup Image/Video Files   *
 *******************************/
var welcome_image_url = DIR + 'images/Welcome.png';
var encoder_img_url = DIR + 'images/encoder.jpg';
var motor_animation_url = DIR + 'videos/motor_animation.mp4';
var multi_choice_url = DIR + 'images/sized_cuff.png';
var hokeypokey_out_url = DIR + 'images/hokeypokey_out.png';
var hokeypokey_in_url = DIR + 'images/hokeypokey_in.png';
var hokeypokey_rot1_url = DIR + 'images/hokeypokey_rot1.png';
var hokeypokey_rot2_url = DIR + 'images/hokeypokey_rot2.png';
var encoder_moving_part_url = DIR + 'images/moving_part.png';
var encoder_still_part_url = DIR + 'images/still_part.png';
var encoder_motor_url = DIR + 'images/motor_body.png';
var encoder_motor_circle_url = DIR + 'images/motor_circle.png';

var back_plate = new Image();
var center_dial = new Image();
var indicator = new Image();
var shadow = new Image();
var mask = new Image();
back_plate.src = DIR + 'images/back_plate.png';
indicator.src = DIR + 'images/Indicator.png';
shadow.src = DIR +  'images/shadow.png';
mask.src = DIR + 'images/mask.png';
center_dial.src = DIR + 'images/center_dial.png';

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, lines_of_text, init, [image,animator,protractor_table,canvas_container]);
var cw = m.cw;
var ch = m.ch;

/*******************************
 *       Graphic Objects       *
 *******************************/
var ballA = {x:25*cw, y:83*ch, fillStyle: 'BlueViolet'};
var ballB = {x:22*cw, y:47*ch, fillStyle: m.robot_color};
var ballC = {x:21*cw, y:18*ch, fillStyle: 'DarkGreen'};
var ballR = 10*ch;

var barL = {axisLeft: 3*cw, axisRight: 12*cw, maxHeight: 87*ch, antiWidth: 0.8*cw, fillStyle: m.robot_color};
var barR = {axisLeft: 15*cw, axisRight: 24*cw, maxHeight: 87*ch, antiWidth: 0.8*cw, fillStyle: 'BlueViolet'};
var barA = {axisLeft: 32*cw, axisRight: 41*cw, maxHeight: 87*ch, antiWidth: 0.8*cw};
var barB = {axisLeft: 44*cw, axisRight: 53*cw, maxHeight: 87*ch, antiWidth: 0.8*cw};
var barC = {axisLeft: 56*cw, axisRight: 65*cw, maxHeight: 87*ch, antiWidth: 0.8*cw};

/**************************
 *   Main Functionality   *
 **************************/
var seq_ind = 0;   // 18
var audio_ind = 0; // 21
async function init() {
    if (seq_ind==0) {
        // 0: Hello, my name is TeachBot. It's nice to meet you. Please stand back so I can show you something.
        m.play(audio_ind);                            // Play opening audio
        await sleep(m.audio_duration[audio_ind]);     // Wait for audio to finish
        audio_ind++;                                  // Increment audio index
        image.style.display = 'initial';              // Display welcome image
    }
    m.pub_cmd(seq_ind);                               // Advance in Py

    // Subscribe to Python Commands
    m.cmd2browser.subscribe(async function(message) {
        if (VERBOSE) console.log('Received: ' + message.data);

        /*/ Check to make sure the counter is synced with the message data
        if (seq_ind!=message.data) {
            let err_msg = 'Error: message.data = ' + message.data + ' but seq_ind = ' + seq_ind;
            console.log(err_msg);
            throw new Error(err_msg);
        }*/
        seq_ind = message.data;

        switch (message.data) {
            case 0:
                // 1: Want to know how I did that? Inside my arm I have a bunch of electronic things called electric motors. They let me move my arm, like how muscles move yours. Here, take a look.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                m.displayOff();
                animator.style.display = 'initial';
                animator.play();

                m.pub_cmd(++seq_ind);
                break;

            case 1:
                m.displayOff();
                canvas_container.style.display = 'initial';
                run_odometer = true;
                canvas_obj.width = canvas_obj.width;
                init_odometer(m.ctx);
                // 2: Just by looking, how many motors do you think I have in my arm? To input your answer, wait for me to move my arm. Next, turn the scroll wheel on my arm to scroll through numbers. Then press on the scroll wheel to select your answer.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 2:
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Rx: ' + message.data);
                    wheel_val+= message.data;
                });
                m.scroll_wheel_button_receiver.subscribe(async function(message) {
                    if (VERBOSE) console.log('Scroll wheel button pressed.');
                    m.numeric_topic.unsubscribe();
                    m.numeric_topic.removeAllListeners();
                    m.scroll_wheel_button_receiver.unsubscribe();
                    m.scroll_wheel_button_receiver.removeAllListeners();
                    cancel_odometer();
                    m.displayOff(true);
                    // 3: Step back and we'll count them together.
                    m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                    m.pub_cmd(++seq_ind);
                });

                break;
            
            case 3:
                image.src = welcome_image_url;
                image.style.display = 'initial';
                // 4: Pay close attention while I move each motor individually. Try to count how many motors I have.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 4:
                // 5: How many motors did you count?
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                m.displayOff(true);
                canvas_container.style.display = 'initial';
                run_odometer = true;
                init_odometer(m.ctx);

                m.pub_cmd(++seq_ind);
                break;

            case 5:
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Rx: ' + message.data);
                    wheel_val+= message.data;
                });
                m.scroll_wheel_button_receiver.subscribe(async function(message) {
                    if (VERBOSE) console.log('Scroll wheel button pressed.');
                    if (wheel_val===JOINTS) {
                        m.numeric_topic.unsubscribe();
                        m.numeric_topic.removeAllListeners();
                        m.scroll_wheel_button_receiver.unsubscribe();
                        m.scroll_wheel_button_receiver.removeAllListeners();
                        cancel_odometer();
                        // 6: Good job! Now, please stand back so I can show you something else.
                        m.play(audio_ind); await sleep(m.audio_duration[audio_ind]);audio_ind+=2;
                        m.displayOff(true);
                        image.style.display = 'initial';
                        m.pub_cmd(++seq_ind);
                    } else {
                        // 7: Not quite. Remember that each motor can move in two directions: clockwise and counterclockwise.
                        m.play(++audio_ind); await sleep(m.audio_duration[audio_ind--]);
                    }
                });
                break;

            case 6:
                // 8: Like most of my human coworkers, I also have a shoulder, an elbow, and a wrist. In fact, my arm has seven motors! For simplicity, I'll call this one my shoulder,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 7:
                // 9: this one my elbow,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 8:
                // 10: and this one my wrist.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 9:
                m.displayOff(true);
                
                // 11: But how do I tell where my arm is? How do I know how far I've rotated each motor? I'm going to unlock my shoulder. Try pushing my arm. I'll display how much I think you've turned it on the screen. When you're finished, press the scroll wheel button to let me know.
                m.play(audio_ind++);

                canvas_container.style.display = 'initial';
                draw_bar(m.ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.data);
                    m.ctx.clearRect(0,0,100*cw,100*ch);
                    draw_bar(m.ctx, -message.data+170, 190, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                });

                m.pub_cmd(++seq_ind);
                break;

            case 10:
                m.numeric_topic.unsubscribe();
                m.numeric_topic.removeAllListeners();
                m.displayOff(true);

                // 12: Let me show you how I know. Please stand back.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 11:
                canvas_container.style.display = 'initial';
                m.ctx.clearRect(0,0,100*cw,100*ch);
                encoder = draw_encoder(m.ctx, encoder_moving_part_url, encoder_still_part_url, encoder_motor_url, encoder_motor_circle_url);
                // 13: This device is called an encoder. Every motor in my arm has one. These encoders can measure the angles of each of my motors.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 12:
                clearInterval(encoder);
                // 14: Now I'm going to lock my shoulder and unlock my wrist. Try pushing my hand. When you're done, press the scroll wheel button.
                m.play(audio_ind++);
                m.displayOff(true);
                canvas_container.style.display = 'initial';
                draw_bar(m.ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                
                m.displayOff(true);
                canvas_container.style.display = 'initial';
                draw_bar(m.ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.data);
                    m.ctx.clearRect(0,0,100*cw,100*ch);
                    draw_bar(m.ctx, message.data+170, 340, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                m.pub_cmd(++seq_ind);
                break;

            case 13:
                m.numeric_topic.unsubscribe();
                m.numeric_topic.removeAllListeners();

                // 15: So far I've only unlocked one motor at a time. Let's try unlocking both my shoulder and wrist motors. Push on my arm again and see what happens. When you're done, press the scroll wheel button.
                m.play(audio_ind++);
                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw_bar(m.ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                draw_bar(m.ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                m.array_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.data);
                    m.ctx.clearRect(0,0,100*cw,100*ch);
                    draw_bar(m.ctx, -message.data[0]+170, 190, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                    draw_bar(m.ctx, message.data[1]+170, 340, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                m.pub_cmd(++seq_ind);
                break;

            case 14:
                m.ctx.clearRect(0,0,100*cw,100*ch);

                // 16: Good! Now, before we go further, how many encoders do you think I have in my arm? Step back so I can move my arm for you to use the scroll wheel.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                run_odometer = true;
                init_odometer(m.ctx);

                m.pub_cmd(++seq_ind);
                break;

            case 15:
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Rx: ' + message.data);
                    wheel_val+= message.data;
                });
                m.scroll_wheel_button_receiver.subscribe(async function(message) {
                    if (VERBOSE) console.log('Scroll wheel button pressed.');
                    if (wheel_val===JOINTS) {
                            m.numeric_topic.unsubscribe();
                            m.numeric_topic.removeAllListeners();
                            m.scroll_wheel_button_receiver.unsubscribe();
                            m.scroll_wheel_button_receiver.removeAllListeners();
                            cancel_odometer();
                            // 17: Good job!
                            m.play(audio_ind); await sleep(m.audio_duration[audio_ind]);audio_ind+=2;
                            m.displayOff(true);
                            image.style.display = 'initial';
                            m.pub_cmd(++seq_ind);
                        } else {
                            // 18: Not quite. If I have one encoder for every motor, how many encoders do I have?
                            m.play(++audio_ind); await sleep(m.audio_duration[audio_ind--]);
                        }
                });
                break;

            case 16:
                m.displayOff(true);

                // 19: Now grab and move my arm again. I've unlocked all of my motors, so my arm might drift around a little. Go ahead and grab it anyway while watching the projection to see what I sense as you move each motor. When you're done, press the scroll wheel button.
                m.play(audio_ind++);
                protractor_table.style.display = 'initial';
                m.array_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.data);
                    for (let p=1; p<=message.data.length; p++) {
                        document.getElementById('protractor' + p).value = '' + (100*message.data[p-1]);
                        let cw = bar_ctx[p-1].canvas.width/100.0;
                        let ch = bar_ctx[p-1].canvas.height/100.0;
                        bar_ctx[p-1].clearRect(0,0,100*cw,100*ch);
                        draw_bar(bar_ctx[p-1], message.data[p-1], 3.15,9*cw,91*cw,50*ch,6*cw, m.robot_color);
                        document.getElementById('bar' + p).value = '' + (100*message.data[p-1]);
                    }
                });

                m.pub_cmd(++seq_ind);
                break;

            case 17:
                m.array_topic.unsubscribe();
                m.array_topic.removeAllListeners();
                
                // 20: Using my encoders, I can measure the angles of all seven of my motors. Please stand back so I can reset my position.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.displayOff(true);
                canvas_container.style.display = 'initial';
                m.ctx.font = Math.round(5*cw) +  'px Raleway';
                draw_ball(m.ctx,ballA.x,ballA.y,ballR,ballA.fillStyle,'A');  // A
                draw_ball(m.ctx,ballB.x,ballB.y,ballR,ballB.fillStyle,'B');  // B

                m.pub_cmd(++seq_ind);
                break;

            case 18:
                // 21: Now let's talk about a new concept, feedback. I've projected some points onto the table. Push my arm so that my shadow is on point B.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 19:
                // 22: Great! Thanks. Now I'm going to move to point B on my own.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 20:
                // 23: Oops! This time I overshot to a new point, C.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                draw_ball(m.ctx,ballC.x,ballC.y,ballR,ballC.fillStyle,'C');

                // 24: Can you correct my position back to point B?
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 21:
                // 25: Thank you! At any point, I know the position of my arm because of my encoder. Here is position A,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 22:
                draw_bar(m.ctx,0.6,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A')
                // 26: here is position B,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 23:
                draw_bar(m.ctx,1,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B')
                // 27: and here is position C.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 24:
                draw_bar(m.ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C')
                // 28: I compare my encoder reading at my current position, C, against the destination encoder position, B. If C is smaller than B, I push right. If C is longer than B, I push left. The "feedback" from my encoders helps me decide which way to go to get to my destination. Now, please stand back.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.displayOff(true);
                image.src = welcome_image_url;
                image.style.display = 'initial';

                m.pub_cmd(++seq_ind);
                break;

            case 25:
                // 29: But that's only one demonstration of feedback. Let me show you another one. Try gently pushing down on my arm. Can you notice the difference?
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 26:
                // 30: Watch the projection as you push my arm. I've put up a little display showing how far you've pushed my arm and how much force I'm using to push back. The further you push my arm, the harder I tell my motors to push back. It should feel a little bit like a rubber band: the more you displace it, the harder it will want to spring back into its original position. When you're done, press the scroll wheel button.
                m.play(audio_ind++);

                m.displayOff(true);
                canvas_container.style.display = 'initial';
                m.array_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log(message.data);
                    draw_goal(m.ctx, 100, message.data[0]*400+100)
                });

                m.pub_cmd(++seq_ind);
                break;

            case 27:
                m.array_topic.unsubscribe();
                m.array_topic.removeAllListeners();
                m.displayOff(true);
                image.style.display = 'initial';

                // 31: Before, I was using the feedback from my encoders to go somewhere new. Now, I'm using feedback to stay in one place. What am I using to know when my arm is moving?
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.displayOff(true);
                canvas_container.style.display = 'initial';
                display_choices(m.ctx, ['Motors','Buttons','Cameras','Encoders','Wheels'], multi_choice_url);

                m.pub_cmd(++seq_ind);
                break;

            case 28:
                m.numeric_topic.subscribe(async function(message) {
                    if (VERBOSE) console.log('Rx: ' + message.data);
                    if (message.data==BUTTON['square']) {
                        m.numeric_topic.unsubscribe();
                        m.numeric_topic.removeAllListeners();
                        // 32: That's right! I measured my arm positions, and created a force to push my arm back. Please stand back so I can reset my arm position.
                        m.play(audio_ind); await sleep(m.audio_duration[audio_ind]);audio_ind+=2;
                        m.pub_cmd(++seq_ind);
                    } else {
                        // 33: Do you remember pushing my arm and I knew how far I moved?
                        m.play(++audio_ind); await sleep(m.audio_duration[audio_ind--]);
                    }
                });
                break;

            case 29:
                m.displayOff(true);
                canvas_container.style.display = 'initial';
                // 34: Now let's talk about why I have so many motors.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 30:
                // 35: Having so many motors is really important. If I can only move my shoulder,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                arc3pt(m.ctx,33*cw,95*ch,27*cw,48*ch,28*cw,2*ch,false);

                m.pub_cmd(++seq_ind);
                break;

            case 31:
                // 36: I can only reach points along this curve. So, if I want to reach this point,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw_ball(m.ctx, 53*cw, 89*ch, ballR, m.robot_color);

                // 37: I'll need to move another motor, too!
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 32:
                // 38: I'm going to move my hand back to where it was and use the projector to show the same arc as before.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                arc3pt(m.ctx,33*cw,95*ch,27*cw,48*ch,28*cw,2*ch,false);

                m.pub_cmd(++seq_ind);
                break;

            case 33:
                // 39: If I want to reach that point, I only need to move one motor, my wrist.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw_ball(m.ctx, 21*cw, 18*ch, ballR, m.robot_color);

                // 40: How many motors will I need to use to reach this new point? When you think you know the answer, push my arm to the point to see if you were right.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 34:
                // 41: I only needed to use one motor, but it was my shoulder, not my wrist. 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 35:
                // 42: With my wrist motor, I could move along this curve, 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.ctx.clearRect(0,0,100*cw,100*ch);
                arc3pt(m.ctx,28*cw,13*ch,38*cw,57*ch,64*cw,46*ch,true);

                m.pub_cmd(++seq_ind);
                break;
                
            case 36:
                // 43: But with my shoulder motor, I can move along this curve. 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                arc3pt(m.ctx,49*cw,94*ch,43*cw,64*ch,42*cw,14*ch,false);

                m.pub_cmd(++seq_ind);
                break;

            case 37:
                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw_ball(m.ctx, 27*cw, 88*ch, ballR, m.robot_color);

                // 44: Let's try another. If I want to move to this point, how many movements will I have to make if I can only move one motor at a time? Just like before, I'm going to unlock my shoulder motor for you to push my hand toward the point. If you want to switch to moving my wrist motor, press the scroll wheel button.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 38:
                // 45: Now I've locked my shoulder and unlocked my wrist. You can switch back and forth at any time by pressing the scroll wheel button.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 39:
                // 46: Great! Now let me try.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 40:
                // 47: I took two steps to get there.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 41:
                // 48: First, I moved this motor, 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 42:
                // 49: then this one. 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 43:
                // 50: I can also travel in four steps 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 44:
                // 51: or eight steps. 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 45:
                // 52: Notice how, the more steps I take, the shorter each step is and the less time I spend using my wrist before switching to my shoulder. If I keep increasing the number of steps, eventually there won't be any time at all between when I am using my wrist and when I am using my shoulder, like this. 
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 46:
                m.ctx.clearRect(0,0,100*cw,100*ch);
                // 53: By combining all of the motions I can make with just one of my motors, I can move my hand freely through a large space!
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                
                // 54: Grab my hand and see how large my range of motion is. I've unlocked all my motors, so my arm might drift around again. Go ahead and grab it anyway. When you're done, press the scroll wheel button.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 47:
                m.displayOff(true);
                image.style.display = 'initial';
                
                // 55: The final lesson of the day is about waypoints. Just like how your phone or computer can save pictures and music in its memory, I use my memory to remember different arm positions. Specifically, whenever you move my arm, I read all of my encoders so that, if I want to, I can return to this exact position. I call that remembered position a "waypoint."
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 48:
                // 56: Let's try an example. I'm going to show you a series of arm positions. After each one, I'll unlock my arm and ask you to move it to that position and press the scroll wheel button.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                image.src = hokeypokey_out_url;

                // 57: Start by pulling my arm so it's extended all the way and matches the image on the projector. When it's in the right position, press the scroll wheel button and I'll record the position as a waypoint.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 49:
                // 58: Good! Now, push my arm in so it's close to my torso. When it's in the right position, press the scroll wheel button.
                m.play(audio_ind++);
                image.src = hokeypokey_in_url;

                m.pub_cmd(++seq_ind);
                break;

            case 50:
                // 59: Now pull my arm back out and press the scroll wheel button.
                m.play(audio_ind++);
                image.src = hokeypokey_out_url;

                m.pub_cmd(++seq_ind);
                break;

            case 51:
                // 60: Now rotate my torso this way and press the scroll wheel button.
                m.play(audio_ind++);
                image.src = hokeypokey_rot1_url;

                m.pub_cmd(++seq_ind);
                break;

            case 52:
                // 61: Finally, rotate my torso that way and press the scroll wheel button.
                m.play(audio_ind++);
                image.src = hokeypokey_rot2_url;

                m.pub_cmd(++seq_ind);
                break;

            case 53:
                // 62: Great job! Now, I'll play back each waypoint. As I move from one waypoint to another, do the same motion with your arm.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                image.src = hokeypokey_out_url;

                // 63: First, you put my arm in.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 54:
                // 64: You put my arm out.
                m.play(audio_ind++);
                image.src = hokeypokey_in_url;

                m.pub_cmd(++seq_ind);
                break;

            case 55:
                // 65: You put my arm in,
                m.play(audio_ind++);
                image.src = hokeypokey_out_url;

                m.pub_cmd(++seq_ind);
                break;

            case 56:
                // 66: And you learned all about, how to set up waypoints and make my motors turn around,
                m.play(audio_ind++);
                image.src = hokeypokey_rot1_url;

                m.pub_cmd(++seq_ind);
                break;

            case 57:
                // 67: And that's what it's all about! High two!
                m.play(audio_ind);
                audio_ind+=3;
                image.src = welcome_image_url;

                m.pub_cmd(++seq_ind);
                break;

            case 58:
                audio_ind-=2;
                // 68: I only have two fingers, so instead of a high five I can only give high two's. High two!
                m.play(audio_ind);
                audio_ind+=2;

                m.pub_cmd(++seq_ind);
                break;

            case 59:
                audio_ind--;
                // 69: Okay,
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 60:
                // 70: As you can probably imagine, playback control can be used for more than just dancing. Today, you recorded my arm positions and played them back to make me do the hokey pokey, but you can also record sequences to make me move equipment, package parts, and more!
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                // 71: Great job! You have completed the first learning module.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;
        }
    });
}