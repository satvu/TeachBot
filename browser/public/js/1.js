'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image, animator]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'feedback';
var start_location = 33;
async function main() {
    m.displayOff();
    image.style.display = 'none';
    m.start([start_seq,start_location]);
}