'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image,animator,protractor_table,canvas_container]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'feedback';
async function main() {
    m.displayOff();
    image.style.display = 'initial';
    m.start([start_seq,31]);
}