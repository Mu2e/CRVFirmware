
#ifndef _WEB_CONST
#define _WEB_CONST


//http://www.w3schools.com/tags/tag_br.asp
//<p> This is normal text - <b>   and this is bold text</b>.</p>         
//The <br> tag inserts a single line break. 

//Add the following meta tag in the source of your web-page. 
//The difference in spelling at the end of the tag is either you use " /> = xml or "> = html.
//<meta http-equiv="Content-Type" content="text/html">
//<plaintext>"

//"<h1 style=\"text-align:center\">CRV FEB mu2e</h1>" //center text



void header1WEB(int prt);
void header1TTY(int prt);
void header0WEB(int prt, int clr);
void header0TTY(int prt, int clr);


/*
Communicating with the Server

As soon as we have a connection to the server (when the open event is fired) 
we can start sending data to the server using the send('your message') method 
on the connection object. It used to support only strings, but in the latest 
spec it now can send binary messages too. To send binary data, you can use 
either Blob or ArrayBuffer object.

// Sending String
connection.send('your message');

// Sending canvas ImageData as ArrayBuffer
var img = canvas_context.getImageData(0, 0, 400, 320);
var binary = new Uint8Array(img.data.length);
for (var i = 0; i < img.data.length; i++) {
  binary[i] = img.data[i];
}
connection.send(binary.buffer);

// Sending file as Blob
var file = document.querySelector('input[type="file"]').files[0];
connection.send(file);
*/

const char WBEG[]=
    {
      //"<!DOCTYPE html>" this line messes up socket closing
        //"<HTTP/1.1 200 OK>\r\n"
        //"<Content-type: text/html>\r\n"
        //"<Content-length: 256>\r\n"
      
        "<html>"
      //"<Connection: Keep-Alive>"
      //"<Keep-Alive: off>"
        "<head>"
      //"<meta http-equiv=\"refresh\" content=\"1\">\r\n"
        "<title>CRV FEB</title>"
      //"<Content-Type: text/html>"
        "</head>"
        "<body bgcolor=\"#CCFFCC\">"
        "<body text= \"#000000\">"
      //"<font color=\"#003399\" size=\"6\"><b><i>Mu2e CRV FEB (TEST BEAM)</i></b></font>"  // bold on '</b>', bold off '<b>'
        "<font color=\"#003399\" size=\"6\"><b><i>Mu2e CRV FEB </i></font>"      // bold on '</b>', bold off '<b>'
        "<body>"
        "<font size=\"3\"</font>"          
        "<pre width=\"88\">"
    };



          
const char WEND[]=
    {
    "</pre>"
    "</body>"
    "</html>"
    };

//    <button type=\"button\">Click Me!</button>\


char XML_Msg[]= 
            {
            "HTTP/1.1 200 OK"\
            "Content-Type: text/html"\
            "Content-Length: 302"\
            "Accept-ranges: bytes"\
            "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\r\n"\
            "<!-- Edited with XML Spy v4.2 -->\r\n"\
            "<note>\r\n"\
            "   '<to>Tove</to>\r\n"\
            "   '<from>Jani</from>\r\n"\
            "   '<heading>Reminder</heading>\r\n"\
            "   '<body>Don\'t forget me this weekend!</body>\r\n"\
            "   '</note>\r\n"\
            "Connection: close"\
};






char WebSides1[]= 
        {
        "<html>\r\n"\
		"<head>\r\n"\
		"<title>Mu2E FEB</title>\r\n"\
                             
		"</head>"\
		"<body bgcolor=\"#3030A0\" text=\"#FFFF00\">\r\n"\
		"<p><b><font color=\"#FFFFFF\" size=\"6\"><i>Mu2E FEB</i></font></b></p>\r\n"\
//"<p><b>This is a dynamic web page </b> <b>easyWEB.</b></p>\r\n"
		"<p><b>Hardware:</b></p>\r\n"\
		"<ul>\r\n"\
		"<li><b>TI RM48L</b></li>\r\n"\
		"<li><b>Wiznet 5300 Ethernet Controller</b></li>\r\n"\
		"</ul>\r\n"\
		"<p><b>A/D Converter Value:</b></p>\r\n"\
		"<table bgcolor=\"#ff0000\" border=\"5\" cellpadding=\"0\" cellspacing=\"0\" width=\"500\">\r\n"\
		"<tr>\r\n"\
		"<td>\r\n"\
		"<table width=\"95%\" border=\"0\" cellpadding=\"0\" cellspacing=\"0\">\r\n"\
		"<tr><td bgcolor=\"#00ff00\">&nbsp;</td></tr>\r\n"\
		"</table>\r\n"\
		"</td>\r\n"\
		"</tr>\r\n"\
		"</table>\r\n"\
		"<table border=\"0\" width=\"500\">\r\n"\
		"<tr>\r\n"\
		"<td width=\"20%\">0V</td>\r\n"\
		"<td width=\"20%\">0,5V</td>\r\n"\
		"<td width=\"20%\">1V</td>\r\n"\
		"<td width=\"20%\">1,5V</td>\r\n"\
		"<td width=\"20%\">2V</td>\r\n"\
		"</tr>\r\n"\
		"</table>\r\n"\
		"<p><b>MCU Temperature:</b></p>\r\n"\
		"<table bgcolor=\"#ff0000\" border=\"5\" cellpadding=\"0\" cellspacing=\"0\" width=\"500\">\r\n"\
		"<tr>"\
		"<td>"\
		"<table width=\"ADA%\" border=\"0\" cellpadding=\"0\" cellspacing=\"0\">\r\n"\
		"<tr><td bgcolor=\"#00ff00\">&nbsp;</td></tr> \r\n"\
		"</table>\r\n"\
		"</td>\r\n"\
		"</tr>\r\n"\
		"</table>\r\n"\
		"<table border=\"0\" width=\"500\">\r\n"\
		"<tr>\r\n"\
		"<td width=\"20%\">20°C</td>\r\n"\
		"<td width=\"20%\">25°C</td>\r\n"\
		"<td width=\"20%\">30°C</td>\r\n"\
		"<td width=\"20%\">35°C</td>\r\n"\
		"<td width=\"20%\">40°C</td>\r\n\n"\
		"</tr>"\
		"</table>"\
      //"<input type=\"text\" name=\"input\" value=\"Type here\">"\

        "<!-- Simple form which will send a POST request -->"
        "<form action=\"\" method=\"post\">"
          "<label for=\"POST-name\">Cmd: </label>"
          "<input id=\"POST-name\" type=\"text\" name=\"name\" maxlength=\"20\" placeholder=\"Enter Command\">"
          "<input type=\"submit\" value=\"Send\">"
          "<input type=\"hidden\" name=\"action\" value=\"addRunner\" id=\"action\">"
          "<input type=\"reset\" value=\"Reset!\">"
        "</form>"
        "<input type=\"button\" value=\"Close this window\" onclick=\"src;\">"
          
        //"<script src=\"/js/closeSocket.js\"></script>"
		"</body>"
  "<script>"
        "myWebSocket.send(\"Hello WebSockets!\");"
        "myWebSocket.close();"          
  "</script>"
		"</html>"
        };



char WebSides1a[]= 
{
  "<!DOCTYPE html>"
  "<meta charset=\"utf-8\" />"
  "<title>WebSocket Test</title>"
  
  "<script"
//        "myWebSocket.send(\"Hello WebSockets!\");"
        "myWebSocket.close();"          
  "</script>"

   "<h2>WebSocket Test</h2>"
   "<div id=\"output\"></div> "
  };




char WebSides1b[]= 
{
  "<!DOCTYPE html>"
  "<meta charset=\"utf-8\" />"
  "<title>WebSocket Test</title>"

  "<script language=\"javascript\" type=\"text/javascript\">"
  "function init();"
   "myWebSocket.send(\"Hello WebSockets!\");"
  "myWebSocket.close();"          
  "</script>"

   //"<h2>WebSocket Test</h2>"
   //"<div id=\"output\"></div> "
  };



char WebSides999[]= 
{
  "<!DOCTYPE html>"
  "<meta charset=\"utf-8\" />"
  "<title>WebSocket Test</title>"
  
  "<script language=\"javascript\" type=\"text/javascript\">"
  
//"var wsUri = \"ws://echo.websocket.org/\"; var output;"
  "var wsUri = \"ws://131.225.52.182/\"; var output;"
  
  "function init()"
  "{ output = document.getElementById(\"output\"); testWebSocket(); }"
  
  "function testWebSocket()"
  "{websocket = new WebSocket(wsUri);"
 //"websocket.onopen = function(evt) { onOpen(evt)};"
   "websocket.onclose = function(evt) { onClose(evt)};"
   "websocket.onmessage = function(evt) { onMessage(evt)};"
   "websocket.onerror = function(evt) { onError(evt)}; } "

  "function onOpen(evt)"
  "{ writeToScreen(\"CONNECTED\");"
    "doSend(\"WebSocket rocks\"); }  "
  
  "function onClose(evt) "
  "{ writeToScreen(\"DISCONNECTED\"); }"

  "function onMessage(evt)"
  "{ writeToScreen('<span style=\"color: blue;\">RESPONSE: ' + evt.data+'</span>');"
    "websocket.close(); }"

  "function onError(evt)"
  "{ writeToScreen('<span style=\"color: red;\">ERROR:</span> ' + evt.data); }"

  "function doSend(message)"
  "{ writeToScreen(\"SENT: \" + message);"
    "websocket.send(message); }"

  "function writeToScreen(message) "
  "{ var pre = document.createElement(\"p\");"
  "pre.style.wordWrap = \"break-word\"; pre.innerHTML = message; output.appendChild(pre); }"
  
  "window.addEventListener(\"load\", init, false);"
  "</script>"
    
    
  "<h2>WebSocket Test</h2>"
  "<div id=\"output\"></div> "
  };

#endif
