var express = require('express');
var path = require('path');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

var html = require('./routes/html')
var audio = require('./routes/audio');
var js = require('./routes/js');
var text = require('./routes/text');
var images = require('./routes/images');
var css = require('./routes/css');
var app = express();

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'pug');

// uncomment after placing your favicon in /public
//app.use(favicon(path.join(__dirname, 'public', 'favicon.ico')));
app.use(logger('dev'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/html',html)
app.use('/audio', audio);
app.use('/js', js);
app.use('/text',text);
app.use('/images', images);
app.use('/css', css);

//app.get all the teachbot required files
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/home.html'));
})
app.get('/home', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/home.html'));
})
app.get('/selectmodules', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/home_modules.html'));
})
app.get('/robotconfiguration', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/robot_configuration.html'));
})
app.get('/cuffinteraction', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/cuff_interaction.html'));
})
app.get('/test', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/test_module_page.html'));
})
app.get('/module1', function(req,res) {
	res.sendFile(path.join(__dirname + '/public/html/1.html'));
});
app.get('/module2', function(req,res) {
	res.sendFile(path.join(__dirname + '/public/html/2.html'));
});
app.get('/module3', function(req,res) {
	res.sendFile(path.join(__dirname + '/public/html/3.html'));
});

app.get('/urTest', function(req, res){
  res.sendFile(path.join(__dirname + '/public/html/urTest.html'));
});

app.get('/table', function(req, res) {
  res.sendFile(path.join(__dirname + '/public/html/table.html'));
});

app.get('/module2_brief', function(req,res){
	res.sendFile(path.join(__dirname + '/public/html/module2_brief.html'))
});

app.get('/module4', function(req,res){
  res.sendFile(path.join(__dirname + '/public/html/4.html'))
})
// catch 404 and forward to error handler
app.use(function(req, res, next) {
  var err = new Error('Not Found');
  err.status = 404;
  next(err);
});

// error handler
app.use(function(err, req, res, next) {
  // set locals, only providing error in development
  res.locals.message = err.message;
  res.locals.error = req.app.get('env') === 'development' ? err : {};

  // render the error page
  res.status(err.status || 500);
  res.render('error');
});


module.exports = app;