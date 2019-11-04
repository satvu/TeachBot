'use strict'

const module_num = 2;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image,canvas_container]);
var cw = m.cw;
var ch = m.ch;

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = '1Dof';
async function main() {
    m.displayOff();
    canvas_container.style.display = 'initial';
    m.start([start_seq,0]);
}
/*
var seq_ind = 0;
var audio_ind = 0;
async function init() {
    m.displayOff();
    canvas_container.style.display = 'initial';

    if (seq_ind==0) {
        // 0: Today, we will be learning about the gripper. Many of today's activities will involve the boxes and bins on the table.
        m.play(audio_ind);                            // Play opening audio
        await sleep(m.audio_duration[audio_ind]);     // Wait for audio to finish
        audio_ind++;                                  // Increment audio index'

        m.ctx.clearRect(0,0,100*cw,100*ch);
        draw('box',1);

        // 1: So, let's get started. I'm going to grab Box 1 with my hands.
        m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
    }
    m.pub_cmd(seq_ind);                               // Advance in Py

    // Subscribe to Python Commands
    m.cmd2browser.subscribe(async function(message) {
        if (VERBOSE) console.log('Received: ' + message.data);

        switch(message.data){
            case 0:
                // 2: Oh, that's embarrassing. Let me try again. I think I know what I did wrong. Would you please move the box back into the designated area? Press the scroll wheel on my arm when you've done so.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 1:
                // 3: The first important concept to learn is the one I just messed up: orientation. As you saw, my hand can't grab the object like this. But...
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                // 4: ...if I change the orientation of my hand, I can pick up this object.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 2:
                // 5: Now you try. I have moved my hand over Box 2. Can you rotate my wrist so I can pick up Box 2? When you're ready for me to pick up the box, press the scroll wheel button.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                // 6: Alright. Let’s see.
                m.play(audio_ind++);
                
                m.pub_cmd(++seq_ind);
                break;

            case 3:
                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw('box',2);
                // 7: Great! You helped me change my gripper's orientation so that I could pick up the box. Now, help me pick up Box 3. This time, my gripper is already in the correct orientation, but not in the correct position. Can you move my hand so that it can pick up Box 3? When you're ready for me to pick up the box, press the scroll wheel button.
                m.play(audio_ind++);

                m.pub_cmd(++seq_ind);
                break;

            case 4:
                // 8: Oops, I couldn’t pick it up like that. This time, try to get the entire width of the box in my grasp.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;

            case 5:
                // 9: Great! Before, you helped me change my gripper's orientation so that I could pick up Box 2. This time, you helped me change my gripper's position so that I could pick up Box 3.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind+=2]);

                m.pub_cmd(++seq_ind);
                break;

            case 6:
                // 10: Oops, I couldn’t pick it up like that. This time, try to get the entire width of the box in my grasp.
                m.play(audio_ind+1);
                audio_ind--;

                m.pub_cmd(--seq_ind);
                break;

            case 7:
                // 11: Now, help me pick up Box 4. If you press the gray button on my wrist, you can change my arm's position. If you press the white button on my wrist, you can change my arm's orientation. Feel free to switch between the two modes to figure out the difference between the position and orientation. When you're ready for me to pick up Box 4, press the scroll wheel button.
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);
                
                m.ctx.clearRect(0,0,100*cw,100*ch);
                draw('bin',1);

                // 12: Great job! 
                m.play(audio_ind); await sleep(m.audio_duration[audio_ind++]);

                m.pub_cmd(++seq_ind);
                break;
            
            case 8:
                audio_ind++;
                // 13: Oops, I couldn’t pick it up like that. This time, try to get the entire width of the box in my grasp.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 9:
                audio_ind++;
                // 14: Now that you've mastered picking up objects with a gripper, let's try placing this box into a bin. It may seem like a simple task, but I can't do it in just one motion. If I try to move the box straight into the bin, I'll collide with the side of the bin, like this.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 10:
                audio_ind++;
                // 15: Can you move the bin back where it was? When it's ready, press the scroll wheel button.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 11:
                audio_ind++;
                // 15: Now let’s try moving this 3rd box into the 2nd bin. Can you move my hand so its position is over the object? Use the gray and white buttons on my wrist to switch between position and orientation modes. Press the scroll wheel to continue.
                m.ctx.clearRect(0,0,100*cw,100*ch);
                m.ctx.strokeRect(480, 450, 110, 40); //box3
                m.play(audio_ind);

                m.pub_cmd(++seq_ind);
                break;

            case 12:
                audio_ind++;
                // 16: Let me try again. This time, instead of trying to move the box straight into the bin, I'll add a couple of steps in the middle. Help me set up a series of waypoints so that I can put the box into the bin. First, raise the box. Don't worry about moving the box directly over the bin. For now, just move it a few inches off the table, higher than the height of a bin. When you're done, press the scroll wheel button.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 13:
                audio_ind++; //should equal 16
                //console.log("Debug message, audio_ind should be 16:", audio_ind)
                // 17; Oops, I couldn't pick it up like that. Try to move my arm again, using the controls from before.
                m.play(audio_ind);

                m.pub_cmd(++seq_ind);
                break;

            case 14:
                audio_ind = 15;
                // 17: Next, move the box over the bin. Don't try to change the height or orientation of the box. Simply position it over the bin. When you're done, press the scroll wheel button.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);
                m.pub_cmd(++seq_ind);
                await sleep( m.audio_duration[audio_ind]); //may cause errors later, not really sure why this guy is here
                
                m.pub_cmd(++seq_ind)
                break;

            case 15:
                audio_ind = 17; //this is stressing me out
                // 18: Now that I've grasped the object, do you think I can move it straight into the bin? Press the X button if you think I can, and the Square button if you think I can't. Before you answer, take a second and think about what you would do to move the box into the bin.
                m.ctx.clearRect(0,0,100*cw,100*ch);
                m.ctx.strokeRect(550, 300, 150, 80); //bin2
                m.play(audio_ind);
                m.numeric_topic.subscribe(async function(message){ 
                    if(message.data == BUTTON['square']){
                        //Correct
                        m.pub_cmd(++seq_ind); 
                        m.numeric_topic.unsubscribe()
                        m.numeric_topic.removeAllListeners();

                    } else if(message.data == 'Button \'X\': OFF'){
                        //Incorrect
                        seq_ind = 22; //UPDATE ME when seq nums change//hardcoding numbers do to jumping, to the oh no case
                        audio_ind = 24; //UPDATE ME when seq nums change
                        m.pub_cmd(16);
                        m.numeric_topic.unsubscribe()
                        m.numeric_topic.removeAllListeners();
                    }
                })
                break;

            case 16:
                audio_ind++;
                // 18: I don't think that's high enough. Move my arm a little bit higher and press the scroll wheel.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(seq_ind);
                break;

            case 17:
                audio_ind++;
                // 19: Finally, we can lower the box into the bin. Make sure the box is low enough so that it doesn't drop too far, then press the scroll wheel button to open my gripper.
                m.play(audio_ind);
                //nothing published, may be the source of errors later

                m.pub_cmd(seq_ind);
                break;
                    
            case 18:
                audio_ind++;
                //21: If I want to place the object in the bin, I’ll have to do something different. First, I could lift up the box. Then...
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 19:
                audio_ind++;
                // 20: Great job! Now let's try moving Box 3 into the second bin. Move my arm so that it is positioned over Box 3. Press the scroll wheel when you're done.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 20:
                audio_ind++;
                // 21: Next, lower my hand so that Box 3 is within the gripper's grasp and press the scroll wheel to close the gripper.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 21:
                audio_ind++;
                // 22: Please stand back so I can confirm.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(++seq_ind);
                break;

            case 22:
                audio_ind++;
                // 23: Good! Now, raise my arm so that the box is high enough to move over the bin. Just like last time, don't worry about moving the box over the bin. Just raise it off the table. When you're done, press the scroll wheel button.
                m.play(audio_ind);
                m.scroll_wheel_button_receiver.subscribe(async function(message){ 
                    m.pub_cmd(13);
                    m.scroll_wheel_button_receiver.unsubscribe()
                    m.scroll_wheel_button_receiver.removeAllListeners();

                });
                break;

            case 23:
                // 24: I wasn't able to pick it up. Try again. Make sure to get the entire width of the box in my grasp.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);
                audio_ind++;
                // 25: This time, the orientation of the box is different than the orientation of the bin. Like before, press the gray button on my wrist to change my arm's position and the white button on my wrist to change my arm's orientation. When the box is positioned over the bin, press the scroll wheel button.
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);
                audio_ind++;
                // 28: Now, let's say I want to move the box back.
                m.ctx.clearRect(0,0,100*cw,100*ch);
                m.ctx.strokeRect(480, 450, 110, 40); //box3
                m.play(audio_ind);
                await sleep( m.audio_duration[audio_ind]);

                m.pub_cmd(seq_ind++);
                break;
        }
    });
}
*/