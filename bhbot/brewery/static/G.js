/*

        NOTICE: THERE IS **NOTHING** IN THIS FILE FOR YOU TO EDIT!

                     YES, THIS DOES INDEED MEAN YOU!!!

                              NOW GO AWAY.


        Copyright 2007, James Melanson
                        james_melanson@yahoo.ca
        
                        jsgraph.js  V1.03      January 3rd, 2007

        All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

        What this means is:

          1.) If you are creating free software that is distributed for free,
              you can use this library for free.

          2.) If you are going to use this library in an application you will
              be selling OR are creating the application for a Commercial
              application OR are creating the application for which you are
              receiving compensation (money, product or employment), then 
              you must purchase a Commercial License for this library for
              US$25. Contact me at the email above for more details.

*/

var JmOoJpJSGraphNumVal = 0;

function JSGraph () {

  JmOoJpJSGraphNumVal++;
  this.id = JmOoJpJSGraphNumVal;

  this.bars = new Array();
  this.trellis = new Array();
  this.lineplots = new Array();

  this.barcounter = 0;
  this.graphTypeValue = 'vertical bar'; //horizontal bar; line plot
  this.graphMaxValue = new Number(0);
  this.gLPO = new Number(30);

  this.prefixJSGraphYBar       = 'JSGraphYBar_'+this.id+'_';
  this.prefixJSGraphYBarShadow = 'JSGraphYBarShadow_'+this.id+'_';
  this.prefixJSGraphXBar       = 'JSGraphXBar_'+this.id+'_';
  this.prefixJSGraphXBarShadow = 'JSGraphXBarShadow_'+this.id+'_';

  this.shadowLightSource = '';
  this.yTextNudgeDownValue = new Number(0);
  this.lineplotCounter = new Number(1);

  this.backgroundColour = '';
  this.backgroundImage = '';
  this.backgroundImageRepeat = 'repeat';

  this.usePlusMarker = false;
  this.trellisOn = false;
  this.yTicksOn = false;
  this.xTicksOn = false;

  this.xAxWidth = new Number(4);
  this.xAxLength = new Number(200);
  this.xAxColour = '#000000';
  this.xAxTop = new Number(0);
  this.xAxLeft = new Number(0);

  this.yAxWidth = new Number(4);
  this.yAxLength = new Number(200);
  this.yAxColour = 'Black';
  this.yAxTop = new Number(0);
  this.yAxLeft = new Number(0);

  this.MG = function() {
    //this is the method that calls the routine to assemble and print the graph.
    _print_graph(this);
  }
  this.verticalBarGraph = function() {
    this.graphTypeValue = 'vertical bar';
  }
  this.horizontalBarGraph = function() {
    this.graphTypeValue = 'horizontal bar';
  }
  this.lightSource = function() {
    if(arguments[0].length == 2) {
      var checkVal = new Array('NW','NE','SW','SE');
      var val;
      for(val in checkVal) {
        if(arguments[0] == checkVal[val]) {
          this.shadowLightSource = arguments[0];
        }
      }
    }
  }
  this.usePlus = function() {
    this.usePlusMarker = true;
  }
  this.yAx = function () {
    //arguments[0] = width
    //arguments[1] = length
    //arguments[2] = colour
    this.yAxWidth = arguments[0];
    this.yAxLength = arguments[1];
    if(arguments[2]) {
      this.yAxColour = arguments[2];
    }
  }
  this.xAx = function () {
    //arguments[0] = width
    //arguments[1] = length
    //arguments[2] = colour
    this.xAxWidth = arguments[0];
    this.xAxLength = arguments[1];
    if(arguments[2]) {
      this.xAxColour = arguments[2];
    }
  }

  this.yAddTicks = function() {
    //arguments[0] = spacing (every X units)
    //arguments[1] = colour
    //arguments[2] = size;
    this.yTicksOn = true;
    this.yTicksSpacing = new Number(5);
    this.yTicksColour = '#000000';
    this.yTicksSize = new Number(2);
    if(arguments[0]) {
      this.yTicksSpacing = arguments[0];
      if(arguments[1].length > 0) {
        this.yTicksColour = arguments[1];
      }
      if(arguments[2]) {
        if(_isNumber(arguments[2])) {
          this.yTicksSize = arguments[2];
        } else {
          window.alert('You did not specify a valid argument for the Y-Axis Tick size.');
        }
      }
    }
  }
  this.xAddTicks = function() {
    //arguments[0] = spacing (every X units)
    //arguments[1] = colour
    //arguments[2] = size;
    this.xTicksOn = true;
    this.xTicksSpacing = new Number(5);
    this.xTicksColour = '#000000';
    this.xTicksSize = new Number(2);
    if(arguments[0]) {
      this.xTicksSpacing = arguments[0];
      if(arguments[1].length > 0) {
        this.xTicksColour = arguments[1];
      }
      if(arguments[2]) {
        if(_isNumber(arguments[2])) {
          this.xTicksSize = arguments[2];
        } else {
          window.alert('You did not specify a valid argument for the X-Axis Tick size.');
        }
      }
    }
  }
  this.T = function() {
    //arguments[0] = spacing
    //arguments[1] = size
    //arguments[2] = color
    //arguments[3] = text
    //arguments[4] = font-family
    //arguments[5] = font-size
    //If only ONE trellis is added, then the system auto repeats while ignoring text etc.
    //If MORE than one trellis is added, then it is user defined.
    if(!arguments[1]) {arguments[1] = ''}
    if(!arguments[2]) {arguments[2] = ''}
    if(!arguments[3]) {arguments[3] = ''}
    if(!arguments[4]) {arguments[4] = ''}
    if(!arguments[5]) {arguments[5] = ''}
    if(this.trellisOn == false) {
      this.trellisOn = true;
      this.trellisSpacing = new Number(10);
      this.trellisSize = new Number(2);
      this.trellisColour = '#000000';
    }
    this.trellis.push(arguments[0]+':'+arguments[1]+':'+arguments[2]+':'+arguments[3]+':'+arguments[4]+':'+arguments[5]);
  }
  this.yAddText = function () {
    //arguments[0] = Text
    //arguments[1] = FontFamily
    //arguments[2] = Colour
    //arguments[3] = Size
    this.yAxTextFontFamily = 'verdana,helvetical,sans-serif';
    this.yAxTextFontColour = '#000000';
    this.yAxTextFontSize = '9pt';
    this.yAxTextValue = arguments[0];
    if(arguments[1]) {
      this.yAxTextFontFamily = arguments[1];
    }
    if(arguments[2]) {
      this.yAxTextFontColour = arguments[2];
    }
    if(arguments[3]) {
      this.yAxTextFontSize = arguments[3];
    }
  }
  this.yTextNudgeDown = function() {
    this.yTextNudgeDownValue = arguments[0];
  }
  this.xAddText = function() {
    //arguments[0] = Text
    //arguments[1] = FontFamily
    //arguments[2] = Colour
    //arguments[3] = Size
    this.xAxTextFontFamily = 'verdana,helvetica,sans-serif';
    this.xAxTextFontColour = '#000000';
    this.xAxTextFontSize = '9pt';
    this.xAxTextValue = arguments[0];
    if(arguments[1]) {
      this.xAxTextFontFamily = arguments[1];
    }
    if(arguments[2]) {
      this.xAxTextFontColour = arguments[2];
    }
    if(arguments[3]) {
      this.xAxTextFontSize = arguments[3];
    }
  }
  this.BG = function() {
    if(arguments[0]) {
      if(arguments[0].indexOf('.') > 0) {
        this.backgroundImage = arguments[0];
        this.backgroundColour = '';
        if(arguments[1]) {
          if(arguments[1] == 'no-repeat') {
            this.backgroundImageRepeat = 'no-repeat';
          }
        }
      } else {
        this.backgroundColour = arguments[0];
        this.backgroundImage = '';
      }
    } else {
      window.alert('You passed a null value to the BG method, you must specify an image URL or a HEX colour.');
    }
  }
  this.P = function() {
    //arguments[0] = Y Axis
    //arguments[1] = Pensize
    //arguments[2] = Colour
    //arguments[3] = UseMarker
    //arguments[4] = label
    //arguments[5] = font family
    //arguments[6] = font colour
    //arguments[7] = font size
    if(!arguments[1]) {arguments[1] = ''}
    if(!arguments[2]) {arguments[2] = ''}
    if(!arguments[3]) {arguments[3] = ''}
    if(!arguments[4]) {arguments[4] = ''}
    if(!arguments[5]) {arguments[5] = ''}
    if(!arguments[6]) {arguments[6] = ''}
    if(!arguments[7]) {arguments[7] = ''}
    this.graphTypeValue = 'line plot';
    this.lineplots.push(this.lineplotCounter+':'+arguments[0]+':'+arguments[1]+':'+arguments[2]+':'+arguments[3]+':'+arguments[4]+':'+arguments[5]+':'+arguments[6]+':'+arguments[7]);
  }
  this.NP = function() {
    this.lineplotCounter++;
  }
  this.pieChart = function() {
    this.pieWidth = new Number(arguments[0]);
    this.pieHeight = new Number(this.pieWidth);
    this.pieLeft = new Number( 0 );
    this.pieTop = new Number( 0 );
    this.gLPO = 0;
    this.pieDivColour = '#000000';
    this.piePercentage = new Array()
    this.pieColour = new Array();
    this.pieLabel = new Array();
    this.graphTypeValue = 'pie';
  }
  this.addPieSlice = function() {
    if(arguments[0]) {
      this.piePercentage.push(arguments[0]);
      //Yes, because of the shifting we are always testing the zero index.
      if(arguments[1]) {
        this.pieColour.push(arguments[1]);
      } else {
        this.pieColour.push('null');
      }
      //arguments[2] = label text
      //arguments[3] = label font
      //arguments[4] = label colour
      //arguments[5] = label size
      if(!arguments[2]) {arguments[2] = ''}
      if(!arguments[3]) {arguments[3] = ''}
      if(!arguments[4]) {arguments[4] = ''}
      if(!arguments[5]) {arguments[5] = ''}
      this.pieLabel.push(arguments[2]+':'+arguments[3]+':'+arguments[4]+':'+arguments[5]);
    }
  }
  this.addBar = function() {
    //arguments[0] = uniqueID
    //arguments[1] = units (size)
    //arguments[2] = width (px)
    //arguments[3] = colour
    if(arguments[0].length > 0) {
      if(parseInt(arguments[1]) > 0) {
        if(parseInt(arguments[2]) > 0) {
          if(arguments[3]) {
            var barUnits = arguments[1];
            var newBar = arguments[0]+':'+parseInt(arguments[1])+':'+arguments[2]+':'+arguments[3];
            this.bars.push( newBar );
          } else {
            window.alert('You did not specify a valid colour for the bar identified as '+arguments[0]);
          }
        } else {
          window.alert('You did not specify a WIDTH for the bar identified as '+arguments[0]);
        }
      } else {
        window.alert('You did not specify the value of UNITS for the bar identified as '+arguments[0]);
      }
    } else {
      window.alert('You have added a bar without specifying a unique ID for the bar.');
    }
  }
  this.modifyBar = function() {
    if(arguments[0].length > 0) {
      if(parseInt(arguments[1]) >= 0) {
        var bari = 0;
        for(bari=0;bari<=(this.bars.length-1);bari++) {
          if(this.bars[bari]) {
            var barVal = new Array;
            barVal = this.bars[bari].split(':');
            //barVal[0] = uniqueID
            //barVal[1] = units
            //barVal[2] = width
            //barVal[3] = colour
            if(parseInt(arguments[1]) > this.graphMaxValue) {
              arguments[1] = this.graphMaxValue;
            }
            if(barVal[0] == arguments[0]) {
              this.bars[bari] = barVal[0]+':'+parseInt(arguments[1])+':'+barVal[2]+':'+barVal[3];
            }
            this.resizeBar(arguments[0], parseInt(arguments[1]));
          }
        }
      }
    } else {
      window.alert('You have attempted to modify one of the graphs bars without specifying the bars unique ID');
    }
  }
  this.resizeBar = function () {
    //arguments[0] = uniqueID
    //arguments[1] = new units value
    if(arguments[0]) {
      if(arguments[1]) {
        if(this.graphTypeValue == 'vertical bar') {
          document.getElementById(this.prefixJSGraphYBar+arguments[0]).innerHTML = '';
          if(arguments[1] > this.graphMaxValue) {
            arguments[1] = this.graphMaxValue;
            document.getElementById(this.prefixJSGraphYBar+arguments[0]).innerHTML = '<font style="font-size:9pt;font-weight:bold;">+</font>';
          }
          var newBarTop = parseInt(this.yAxLength - arguments[1] - 1);
          var newBarLength = arguments[1];
          document.getElementById(this.prefixJSGraphYBar+arguments[0]).style.top = newBarTop.toString()+'px';
          document.getElementById(this.prefixJSGraphYBar+arguments[0]).style.height = newBarLength.toString()+'px';
          //this.resizeShadow(arguments[0],newBarLength,newBarTop);
          var shadowOffset = 3;
          if( (this.shadowLightSource == 'NW') || (this.shadowLightSource == 'NE') ) {
            var newShadowTop = parseInt(newBarTop) + shadowOffset;
            var newShadowLength = parseInt(newBarLength) - shadowOffset;
            if(newShadowLength > this.graphMaxValue) {
              newShadowLength = this.graphMaxValue;
              newShadowTop = newBarTop;
            }
            document.getElementById(this.prefixJSGraphYBarShadow+arguments[0]+'s').style.top = newShadowTop.toString()+'px';
            document.getElementById(this.prefixJSGraphYBarShadow+arguments[0]+'s').style.height = newShadowLength.toString()+'px';
          } else {
            var newShadowTop = parseInt(newBarTop) - shadowOffset;
            var newShadowLength = parseInt(newBarLength) + shadowOffset;
            if(newShadowLength > this.graphMaxValue) {
              newShadowLength = this.graphMaxValue;
              newShadowTop = newBarTop;
            }
            document.getElementById(this.prefixJSGraphYBarShadow+arguments[0]+'s').style.top = newShadowTop.toString()+'px';
            document.getElementById(this.prefixJSGraphYBarShadow+arguments[0]+'s').style.height = newShadowLength.toString()+'px';
          }
        } else {
          //Horizontal
          document.getElementById(this.prefixJSGraphXBar+arguments[0]).innerHTML = '';
          var shadowOffset = 3;
          var shadowLength = 0;
          var newBarLength = arguments[1];
          if(newBarLength > this.graphMaxValue) {
            newBarLength = this.graphMaxValue;
            document.getElementById(this.prefixJSGraphXBar+arguments[0]).innerHTML = '<font style="font-size:9pt;font-weight:bold;">+</font>';
          }
          if( (this.shadowLightSource == 'NW') || (this.shadowLightSource == 'SW') ) {
            shadowLength = parseInt(newBarLength) + shadowOffset;
          } else {
            shadowLength = parseInt(newBarLength) - shadowOffset;
          }
          if(shadowLength > this.graphMaxValue) {
            shadowLength = this.graphMaxValue;
          }
          document.getElementById(this.prefixJSGraphXBar+arguments[0]).style.width = newBarLength.toString()+'px';
          document.getElementById(this.prefixJSGraphXBarShadow+arguments[0]+'s').style.width = shadowLength.toString()+'px';
        }
      }
    }
  }
  this.hideBar = function() {
    //arguments[0] = uniqueID
    if(arguments[0]) {
      var barHandle;
      var shadowHandle;
      if(this.graphTypeValue == 'vertical bar') {
        barHandle = this.prefixJSGraphYBar+arguments[0];
        shadowHandle = this.prefixJSGraphYBarShadow+arguments[0]+'s';
      } else {
        barHandle = this.prefixJSGraphXBar+arguments[0];
        shadowHandle = this.prefixJSGraphXBarShadow+arguments[0]+'s';
      }
      document.getElementById(barHandle).style.display = 'none';
      document.getElementById(shadowHandle).style.display = 'none';
    }
  }
  this.unhideBar = function() {
    //arguments[0] = uniqueID
    if(arguments[0]) {
      var barHandle;
      var shadowHandle;
      if(this.graphTypeValue == 'vertical bar') {
        barHandle = this.prefixJSGraphYBar+arguments[0];
        shadowHandle = this.prefixJSGraphYBarShadow+arguments[0]+'s';
      } else {
        barHandle = this.prefixJSGraphXBar+arguments[0];
        shadowHandle = this.prefixJSGraphXBarShadow+arguments[0]+'s';
      }
      document.getElementById(barHandle).style.display = 'block';
      document.getElementById(shadowHandle).style.display = 'block';
    }
  }
}

function _print_graph (oHandle) {
  if(oHandle.graphTypeValue == 'vertical bar') {
    oHandle.graphMaxValue = oHandle.yAxLength;
  } else {
    oHandle.graphMaxValue = oHandle.xAxLength;
  }
  //Create background bar.
  if(oHandle.backgroundImage || oHandle.backgroundColour) {
    var printBackground = '<div id="JSGraphBackground'+this.id+'" style="position:absolute;z-index:1;';
    printBackground += 'left:'+parseInt(oHandle.gLPO+oHandle.yAxWidth)+'px;';
    printBackground += 'top:0px;';
    printBackground += 'width:'+oHandle.xAxLength+'px;';
    printBackground += 'height:'+oHandle.yAxLength+'px;';
    if(oHandle.backgroundImage) {
      printBackground += 'background: url('+oHandle.backgroundImage+') ';
      printBackground += oHandle.backgroundImageRepeat+';';
    } else {
      printBackground += 'background-color:'+oHandle.backgroundColour+';';
    }
    printBackground += '"></div>';
    document.write(printBackground);
    printBackground = '';
  }

  if(oHandle.graphTypeValue != 'pie') {
    var printyAx = '<div id="JSGraphyAx'+this.id+'" style="position:absolute;font-size:1px;z-index:2;';
    printyAx += 'left:'+oHandle.gLPO+'px;';
    oHandle.yAxLeft = oHandle.gLPO;
    printyAx += 'top:0px;';
    oHandle.yAxTop = 0;
    printyAx += 'width:'+oHandle.yAxWidth+'px;';
    printyAx += 'height:'+oHandle.yAxLength+'px;';
    printyAx += 'background-color:'+oHandle.yAxColour+';"></div>';
    document.write(printyAx);
    printyAx = '';

    var printxAx = '<div id="JSGraphxAx'+this.id+'" style="position:absolute;font-size:1px;z-index:2;';
    printxAx += 'left:'+oHandle.gLPO+'px;';
    oHandle.xAxLeft = parseInt(oHandle.gLPO+oHandle.yAxWidth);
    printxAx += 'top:'+oHandle.yAxLength+'px;';
    oHandle.xAxTop = oHandle.yAxLength;
    printxAx += 'width:'+parseInt((oHandle.xAxLength*10)+oHandle.yAxWidth)+'px;'; // JP x 10
    printxAx += 'height:'+oHandle.xAxWidth+'px;';
    printxAx += 'background-color:'+oHandle.xAxColour+';"></div>';
    document.write(printxAx);
    printxAx = '';
  }

  if(oHandle.yAxTextValue) {
    var printyAxLabel = '<div id="JSGraphyAxLabel'+this.id+'" style="position:absolute;left:0px;z-index:5;text-align:center;';
    if(oHandle.yTextNudgeDownValue > 0) {
      printyAxLabel += 'top:'+oHandle.yTextNudgeDownValue+'px;';
    } else {
      printyAxLabel += 'top:0px;';
    }
    if(oHandle.yAxTextFontCSS) {
      printyAxLabel += oHandle.yAxTextFontCSS;
    } else {
      printyAxLabel += 'font-family:'+oHandle.yAxTextFontFamily+';';
      printyAxLabel += 'font-size:'+oHandle.yAxTextFontSize+';';
      printyAxLabel += 'color:'+oHandle.yAxTextFontColour+';';
    }
    printyAxLabel += '">';
    var Ytexti = 0;
    var printBr = 0;
    for(Ytexti=0;Ytexti<oHandle.yAxTextValue.length;Ytexti++) {
      if(oHandle.yAxTextValue.substr(Ytexti,1) == ' ') {
        printyAxLabel += '<br>';
      } else {
        if(printBr) {
          printyAxLabel += '<br>';
        }
        printyAxLabel += oHandle.yAxTextValue.substr(Ytexti,1);
        printBr = 1;
      }
    }
    printyAxLabel += '</div>';
    document.write(printyAxLabel);
    printyAxLabel = '';
  }

  if(oHandle.yTicksOn == true) {
    var iterate = 0;
    var thisTickTop = 1;
    while(thisTickTop > 0) {
      iterate++;
      thisTickTop = parseInt(oHandle.yAxLength - parseInt(iterate * oHandle.yTicksSpacing));
      var printThisTick = '<div id="JSGraphyAxTick'+this.id+'_'+iterate.toString()+'" style="position:absolute;font-size:1px;z-index:2;';
      printThisTick += 'top:'+thisTickTop+'px;';
      printThisTick += 'left:'+parseInt(oHandle.gLPO - 4)+'px;';
      printThisTick += 'height:'+oHandle.yTicksSize+'px;';
      printThisTick += 'width:4px;';
      printThisTick += 'background-color:'+oHandle.yTicksColour+';"></div>';
      document.write(printThisTick);
      printThisTick = '';
    }
  }

  if(oHandle.xAxTextValue) {
    var printxAxLabel = '<div id="JSGraphxAxLabel'+this.id+'" style="position:absolute;z-index:5;text-align:center;';
    printxAxLabel += 'left:'+parseInt(oHandle.gLPO+oHandle.yAxWidth)+'px;';
    if(oHandle.xTicksOn == true) {
      printxAxLabel += 'top:'+parseInt( oHandle.yAxLength + oHandle.xAxWidth + parseInt(oHandle.xTicksSize * 2) )+'px;';
    } else {
      printxAxLabel += 'top:'+parseInt( oHandle.yAxLength + oHandle.xAxWidth )+'px;';
    }
    printxAxLabel += 'width:'+oHandle.xAxLength+'px;';
    if(oHandle.xAxTextFontCSS) {
      printxAxLabel += oHandle.xAxTextFontCSS;
    } else {
      printxAxLabel += 'font-family:'+oHandle.xAxTextFontFamily+';';
      printxAxLabel += 'font-size:'+oHandle.xAxTextFontSize+';';
      printxAxLabel += 'color:'+oHandle.xAxTextFontColour+';';
    }
    printxAxLabel += '">';
    printxAxLabel += oHandle.xAxTextValue+'</div>';
    document.write(printxAxLabel);
    printxAxLabel = '';
  }

  if(oHandle.xTicksOn == true) {
    var thisTickTop = parseInt(oHandle.yAxLength + oHandle.xAxWidth);
    var iterate = 1;
    var thisTickLeft = parseInt(oHandle.gLPO + oHandle.yAxWidth + parseInt(iterate * oHandle.xTicksSpacing) - oHandle.xTicksSize);
    while(thisTickLeft < parseInt(oHandle.gLPO + oHandle.yAxWidth + oHandle.xAxLength) ) {
      var printThisTick = '<div id="JSGraphxAxTick'+this.id+'_'+iterate.toString()+'" style="position:absolute;font-size:1px;z-index:2;';
      printThisTick += 'left:'+thisTickLeft+'px;';
      printThisTick += 'top:'+thisTickTop+'px;';
      printThisTick += 'height:4px;';
      printThisTick += 'width:'+oHandle.xTicksSize+'px;';
      printThisTick += 'background-color:'+oHandle.xTicksColour+';"></div>';
      document.write(printThisTick);
      printThisTick = '';
      iterate++;
      thisTickLeft = parseInt(oHandle.gLPO + oHandle.yAxWidth + parseInt(iterate * oHandle.xTicksSpacing) - oHandle.xTicksSize);
    }
  }

  if(oHandle.trellisOn == true) {
    if(oHandle.trellis.length > 1) {
      //User defined trellis lines, not auto
      //base assembly:
      var trelli = 0;
      for(trelli=0;trelli<oHandle.trellis.length;trelli++) {
        var trellisVal = oHandle.trellis[trelli].split(':');
        var thisHandle = 'JSGraphTrellis'+this.id+'_'+parseInt(trelli+1);
        var printThisTrellis = '<div id="'+thisHandle+'" style="position:absolute;font-size:1px;z-index:5;';
        var printThisTrellisText = '<div id="'+thisHandle+'text" style="position:absolute;z-index:5;';
        if( (oHandle.graphTypeValue == 'vertical bar') || (oHandle.graphTypeValue == 'line plot') ) {
          //vertical bar graph so trellis is horizontal
          printThisTrellis += 'left:'+parseInt(oHandle.gLPO+oHandle.yAxWidth)+'px;';
          printThisTrellis += 'width:'+oHandle.xAxLength+'px;';
          printThisTrellis += 'top:'+(parseInt(oHandle.yAxLength) - parseInt(trellisVal[0]))+'px;';
          if(trellisVal[1]) {
            printThisTrellis += 'height:'+trellisVal[1]+'px;';
          } else {
            printThisTrellis += 'height:'+oHandle.trellisSize+'px;';

          }
          if(trellisVal[3]) {
            printThisTrellisText += 'text-align:right;left:0px;';
            printThisTrellisText += 'top:'+((parseInt(oHandle.yAxLength) - trellisVal[0]) - parseInt(parseInt(trellisVal[5]) / 2))+'px;';
            printThisTrellisText += 'width:'+(parseInt(oHandle.gLPO) - 2)+'px;';
            printThisTrellisText += 'font-family:'+trellisVal[4]+';';
            printThisTrellisText += 'font-size:'+trellisVal[5]+'px;';
            printThisTrellisText += 'color:'+trellisVal[2]+';';
            printThisTrellisText += '">'+trellisVal[3]+'</div>';
          }
        } else {
          //Horizontal - so trellis is vertical
          printThisTrellis += 'top:0px;';
          if(trellisVal[1]) {
            printThisTrellis += 'width:'+trellisVal[1]+'px;';
          } else {
            printThisTrellis += 'width:'+oHandle.trellisSize+'px;';
          }
          printThisTrellis += 'height:'+oHandle.yAxLength+'px;';
          printThisTrellis += 'left:'+(parseInt(oHandle.gLPO) + parseInt(oHandle.yAxWidth) + parseInt(trellisVal[0]) )+'px;';
          if(trellisVal[3]) {
            printThisTrellisText += 'left:'+( (parseInt(oHandle.gLPO) + parseInt(oHandle.yAxWidth) + parseInt(trellisVal[0])) - parseInt((trellisVal[3].length * parseInt(trellisVal[5])) / 4) )+'px;';
            printThisTrellisText += 'top:'+(parseInt(oHandle.yAxLength) + parseInt(oHandle.xAxWidth) + 2)+'px;';
            printThisTrellisText += 'width:'+parseInt(parseInt(trellisVal[5]) * trellisVal[3].length)+'px;';
            printThisTrellisText += 'font-family:'+trellisVal[4]+';';
            printThisTrellisText += 'font-size:'+trellisVal[5]+'px;';
            printThisTrellisText += 'color:'+trellisVal[2]+';';
            printThisTrellisText += '">'+trellisVal[3]+'</div>';
          }
        }
        if(trellisVal[2]) {
          printThisTrellis += 'background-color:'+trellisVal[2]+';"></div>';
        } else {
          printThisTrellis += 'background-color:'+oHandle.trellisColour+';"></div>';
        }
        document.write(printThisTrellis);
        printThisTrellis = '';
        if(trellisVal[3]) {
          document.write(printThisTrellisText);
        }
        printThisTrellisText = '';
      }
    } else {
      var trellisVal = oHandle.trellis[0].split(':');
      var iterate = 0;
      if(oHandle.graphTypeValue == 'vertical bar') {
        //Vertical bar, trellis horizontal from Y-Axis
        var thisTrellisTop = 1;
        while(thisTrellisTop > 0) {
          iterate++;
          thisTrellisTop = parseInt(oHandle.yAxLength - parseInt(iterate * trellisVal[0]));
          var printThisTrellis = '<div id="JSGraphTrellis'+this.id+'_'+iterate.toString()+'" style="position:absolute;font-size:1px;z-index:5;';
          printThisTrellis += 'top:'+thisTrellisTop+'px;';
          printThisTrellis += 'left:'+parseInt(oHandle.gLPO+oHandle.yAxWidth)+'px;';
          printThisTrellis += 'height:'+trellisVal[1]+'px;';
          printThisTrellis += 'width:'+oHandle.xAxLength+'px;';
          printThisTrellis += 'background-color:'+oHandle.trellisColour+';"></div>';
          document.write(printThisTrellis);
          printThisTrellis = '';
        }
      } else {
        //Horizontal bar, trellis vertical from X-Axis
        var thisTrellisLeft = parseInt(parseInt(oHandle.gLPO)+parseInt(oHandle.yAxWidth)+parseInt(trellisVal[0]));
        while(thisTrellisLeft <= parseInt(parseInt(oHandle.gLPO)+parseInt(oHandle.yAxWidth)+parseInt(oHandle.xAxLength)) ) {
          iterate++;
          var printThisTrellis = '<div id="JSGraphTrellis'+this.id+'_'+iterate.toString()+'" style="position:absolute;font-size:1px;z-index:5;';
          printThisTrellis += 'top:0px;';
          printThisTrellis += 'left:'+thisTrellisLeft+'px;';
          printThisTrellis += 'height:'+oHandle.yAxLength+'px;';
          printThisTrellis += 'width:'+trellisVal[1]+'px;';
          printThisTrellis += 'background-color:'+oHandle.trellisColour+';"></div>';
          document.write(printThisTrellis);
          printThisTrellis = '';
          thisTrellisLeft += parseInt(trellisVal[0]);
        }
      }
    }
  }

  if(oHandle.graphTypeValue == 'pie') {
    /* This section deals with the function that draws the
       circle to the screen. It was taken from:

               wz_jsgraphics.js    v. 2.36

       That library is distributed under the GNU Lesser
       General Public License. It was created 3. 11. 2002
       by Walter Zorn (Web: http://www.walterzorn.com )
       Last modified: 21. 6. 2006

       This function was heavily modified with the addition
       of code and modification of behaviour of the existing
       code on January 2nd, 2007 by James Melanson.
    */
    var a = oHandle.pieWidth>>1; var b = oHandle.pieHeight>>1;
    var neV = new Array();
    var seV = new Array();
    var swV = new Array();
    var nwV = new Array();
    var wod = oHandle.pieWidth&1; var hod = (oHandle.pieHeight&1)+1;
    var cx = oHandle.pieLeft+a; var cy = oHandle.pieTop+b;
    var x = 0; var y = b;
    var ox = 0; var oy = b;
    var aa = (a*a)<<1; var bb = (b*b)<<1;
    var st = (aa>>1)*(1-(b<<1)) + bb;
    var tt = (bb>>1) - aa*((b<<1)-1);
    var w; var h;
    while (y > 0) {
      if (st < 0) {
        st += bb*((x<<1)+3);
        tt += (bb<<1)*(++x);
      }
      else if (tt < 0) {
        st += bb*((x<<1)+3) - (aa<<1)*(y-1);
        tt += (bb<<1)*(++x) - aa*(((y--)<<1)-3);
        w = x-ox;
        h = oy-y;
        if (w&2 && h&2) {
          neV.push( (ox+wod+cx)+':'+(-oy+cy)+':1:1' );
          seV.push( (ox+wod+cx)+':'+(oy-1+hod+cy)+':1:1' );
          swV.push( (-x+2+cx)+':'+(oy-1+hod+cy)+':1:1' );
          nwV.push( (-x+2+cx)+':'+(-oy+cy)+':1:1' );

          neV.push( (x-1+wod+cx)+':'+(-y-1+cy)+':1:1' );
          seV.push( (x-1+wod+cx)+':'+(y+hod+cy)+':1:1' );
          swV.push( (-x+1+cx)+':'+(y+hod+cy)+':1:1' );
          nwV.push( (-x+1+cx)+':'+(-y-1+cy)+':1:1' );
        } else {
          neV.push( (ox+wod+cx)+':'+(-oy+cy)+':'+w+':'+h );
          seV.push( (ox+wod+cx)+':'+(oy-h+hod+cy)+':'+w+':'+h );
          swV.push( (-x+1+cx)+':'+(oy-h+hod+cy)+':'+w+':'+h );
          nwV.push( (-x+1+cx)+':'+(-oy+cy)+':'+w+':'+h );
        }
        ox = x;
        oy = y;
      } else {
        tt -= aa*((y<<1)-3);
        st -= (aa<<1)*(--y);
      }
    }
    var fullCircle = new Array();
    var serV = seV.reverse();
    var nwrV = nwV.reverse();
    fullCircle = neV;
    fullCircle.push( (cx+ox+wod)+':'+(cy-oy)+':'+(a-ox+1)+':'+((oy<<1)+hod) )
    for(var serVi=0;serVi<serV.length;serVi++) {
      fullCircle.push(serV[serVi]);
    }
    for(var swVi=0;swVi<swV.length;swVi++) {
      fullCircle.push(swV[swVi]);
    }
    fullCircle.push( (cx-a)+':'+(cy-oy)+':'+(a-ox+1)+':'+((oy<<1)+hod) )
    for(var nwrVi=0;nwrVi<nwrV.length;nwrVi++) {
      fullCircle.push(nwrV[nwrVi]);
    }

    var circp = new Array();
    for(var pci = 0; pci < oHandle.piePercentage.length; pci++) {
      circp.push( Math.ceil(parseInt(fullCircle.length) * parseFloat(oHandle.piePercentage[pci] / 100)) );
    }
    var linePlotPoints = new Array();
    var firstTol = circp.shift();
    var divcount = 0;
    var divcolour = oHandle.pieColour.shift();
    for(var fci = 0; fci < fullCircle.length; fci++) {
      divcount++;
      var plot = fullCircle[fci];
      var fc_parts = new Array();
      fc_parts = plot.split(':');
      if(fci==0) {
        linePlotPoints.push(fc_parts[0]+':'+fc_parts[1]+':0');
      }
      if(divcount == firstTol) {
        firstTol = circp.shift();
        divcolour = oHandle.pieColour.shift();
        divcount = 0;
        linePlotPoints.push(fc_parts[0]+':'+fc_parts[1]+':'+fci);
      }
      if(!divcolour || (divcolour == 'null')) {
        divcolour = oHandle.pieDivColour;
      }
      _make_Circle_Div(fc_parts[0],fc_parts[1],fc_parts[2],fc_parts[3],divcolour);
    }
    var basePlotPointX = oHandle.pieLeft + Math.ceil(parseInt(oHandle.pieWidth) / 2);
    var basePlotPointY = Math.ceil(parseInt(oHandle.pieHeight) / 2);
    for( var lppi = 0; lppi < linePlotPoints.length; lppi++) {
      var nextPlotV = linePlotPoints[lppi];
      if(nextPlotV) {
        var nextPlot = nextPlotV.split(':');
        _draw_line(oHandle, 1, '#000000', basePlotPointX, basePlotPointY, nextPlot[0], nextPlot[1]);
      }
    }
    //Now print the pie slice labels
    var lastPlotPos = 0;
    for(var labeli = 0; labeli < linePlotPoints.length; labeli++) {
      var plotPointV;
      if(linePlotPoints[labeli + 1]) {
        plotPointV = linePlotPoints[labeli + 1];
      } else {
        plotPointV = linePlotPoints[0];
      }
      if(plotPointV) {
        var plotPoint = plotPointV.split(':');
        if(oHandle.pieLabel[labeli]) {
          var plotLabelV = oHandle.pieLabel[labeli];
          var plotLabel = plotLabelV.split(':');
          if(plotLabel[0]) {
            if(!plotLabel[1]) {plotLabel[1] = 'tahoma,sans-serif'}
            if(!plotLabel[2]) {plotLabel[2] = '#000000'}
            if(!plotLabel[3]) {plotLabel[3] = 9}
            if(labeli == (linePlotPoints.length - 1)) {
              fc_plot = (fullCircle.length - 1) - Math.ceil(((fullCircle.length - 1) - lastPlotPos) / 2);
            } else {
              fc_plot = parseInt(plotPoint[2]) - Math.ceil((plotPoint[2] - lastPlotPos) / 2);
            }
            var thisPositV = fullCircle[ fc_plot ];
            if(thisPositV) {
              var thisPosit = thisPositV.split(':');
              var printPlotLabel = '<div id="JSGraphPieLabel'+this.id+'_'+labeli.toString()+'" style="position:absolute;font-size:1px;z-index:6;';
              printPlotLabel += 'top:'+thisPosit[1]+'px;';
              printPlotLabel += 'left:'+thisPosit[0]+'px;';
              printPlotLabel += 'width:'+((plotLabel[0].length * plotLabel[3]) * .6)+'px;';
              printPlotLabel += 'font-family:'+plotLabel[1]+';';
              printPlotLabel += 'color:'+plotLabel[2]+';';
              printPlotLabel += 'font-size:'+plotLabel[3]+'px;';
              printPlotLabel += 'background:transparent;">'+plotLabel[0]+'</div>';
              document.write(printPlotLabel);
              printPlotLabel = '';
            }
          }
        }
        lastPlotPos = plotPoint[2];
      }
    }
  }
  else if(oHandle.graphTypeValue == 'line plot') {
    var lineplotPensize = new Number(1);
    var lineplotColour = '#000000';
    var lineplotMarker = 0;
    if(oHandle.lineplots.length > 1) {
      var pensize;
      var pencolour;
      var basePlot = new Array(8);
      basePlot = oHandle.lineplots.shift();
      var baseVal = basePlot.split(':');
      var plotLineNumber = baseVal[0];
      if(baseVal[1] > oHandle.yAxLength) {
        baseVal[1] = oHandle.yAxLength;
      }
      var baseY = new Number(oHandle.yAxLength - baseVal[1]);
      var baseX = new Number(parseInt(oHandle.gLPO) + parseInt(oHandle.yAxWidth));
      var maxPlots = _get_max_plots(oHandle.lineplots);
      var plotSpacing = new Number( parseInt(oHandle.xAxLength / (maxPlots + 1) * 10) ); // JP 10 x
      var count = 0;
      while(oHandle.lineplots.length > 0) {
        count++;
        var x;
        var plot = oHandle.lineplots.shift();
        if(plot) {
          var plotVal = new Array(8);
          plotVal = plot.split(':');
          if(plotVal[0] != plotLineNumber) {
            plotLineNumber = plotVal[0];
            baseY = parseInt(oHandle.yAxLength - plotVal[1]);
            baseX = parseInt(parseInt(oHandle.gLPO) + parseInt(oHandle.yAxWidth));
            x = baseX;

          } else {
            x = (parseInt(baseX) + parseInt(plotSpacing));
          }
          if(plotVal[1] > oHandle.yAxLength) {
            plotVal[1] = oHandle.yAxLength;
          }
          var y = (parseInt(oHandle.yAxLength) - parseInt(plotVal[1]));
          //var pensize = plotVal[2];
          //if(!pensize) {
          //  pensize = lineplotPensize;
          //}
          //var pencolour = plotVal[3];
          //if(!pencolour) {
          //  pencolour = lineplotColour;
          //}

          if(plotVal[2]) {
            pensize = plotVal[2];
          }
          else if(plotVal[2] == 'undefined') {
            pensize = lineplotPensize;
          } else {
            pensize = lineplotPensize;
          }
          if(plotVal[3]) {
            pencolour = plotVal[3];
          }
          else if(plotVal[3] == 'undefined') {
            pencolour = plotVal[3];
          } else {
            pencolour = lineplotColour;
          }
          _draw_line(oHandle, pensize, pencolour, baseX, baseY, x, y);
          if(plotVal[4]) {
            var boxHandle = 'box'+plotVal[1].toString()+count.toString();
            var printPlotMarker = '<div id="'+boxHandle+'" style="position:absolute;margin:0px;padding:0px;font-size:1px;background-color:'+pencolour+';z-index:6;';
            printPlotMarker += 'left:'+parseInt(x - pensize)+'px;';
            printPlotMarker += 'top:'+parseInt(y - pensize)+'px;';
            printPlotMarker += 'width:'+parseInt(parseInt(pensize) * 3)+'px;';
            printPlotMarker += 'height:'+parseInt(parseInt(pensize) * 3)+'px;';
            printPlotMarker += '"></div>';
            document.write(printPlotMarker);
            printPlotMarker = '';
          }
          if( plotVal[5] && (plotLineNumber == 1) ) {
            if(!plotVal[6]) {
              plotVal[6] = oHandle.lineplotFontFamily;
            }
            if(!plotVal[7]) {
              plotVal[7] = oHandle.lineplotFontColour;
            }
            if(!plotVal[8]) {
              plotVal[8] = oHandle.lineplotFontSize;
            }
            var printPlotText = '<div id="'+boxHandle+'text" style="position:absolute;z-index:6;';
            printPlotText += 'left:'+parseInt( x - parseInt((plotVal[5].length * parseInt(plotVal[8])) / 4) )+'px;';
            printPlotText += 'top:'+parseInt(parseInt(oHandle.yAxLength) + parseInt(oHandle.xAxWidth) + 2)+'px;';
            printPlotText += 'width:'+parseInt(parseInt(plotVal[8]) * plotVal[5].length)+'px;';
            printPlotText += 'font-family:'+plotVal[6]+';';
            printPlotText += 'font-size:'+plotVal[8]+'px;';
            printPlotText += 'color:'+plotVal[7]+';';
            printPlotText += '">'+plotVal[5]+'</div>';
            document.write(printPlotText);
            printPlotText = '';
          }
          baseX = x;
          baseY = y;
        }
      }
    }
  }
  else if(oHandle.graphTypeValue == 'vertical bar') {
    //Vertical bars
    var barSpacing = parseInt(oHandle.xAxLength / (oHandle.bars.length + 1));
    var iterate = 0;
    var bari = 0;
    for(bari=0;bari<=(oHandle.bars.length-1);bari++) {
      if(oHandle.bars[bari]) {
        iterate++;
        var barVal = new Array;
        barVal = oHandle.bars[bari].split(':');
        //barVal[0] = uniqueID
        //barVal[1] = units
        //barVal[2] = width
        //barVal[3] = colour
        //BAR
        var plus;
        var barLength = barVal[1];
        if(barLength > oHandle.graphMaxValue) {
          barLength = oHandle.graphMaxValue;
           plus = '<font style="font-size:9pt;font-weight:bold;">+</font>';
        }
        var printThisBar = '<div id="'+oHandle.prefixJSGraphYBar+barVal[0].toString()+'" style="position:absolute;font-size:1px;text-align:center;border-top:1px outset #F5F5F5;border-right:1px outset #F5F5F5;z-index:10;padding: 0px 0px 0px 0px;';
        printThisBar += 'top:'+parseInt(oHandle.yAxLength - barLength)+'px;';
        printThisBar += 'left:'+(oHandle.gLPO + oHandle.yAxWidth + ( (barSpacing * iterate) - barVal[2] ))+'px;';
        printThisBar += 'width:'+barVal[2]+'px;';
        printThisBar += 'height:'+(barLength - 1)+'px;';
        printThisBar += 'background-color:'+barVal[3]+';">';
        if(oHandle.usePlusMarker == true) {
          if(plus) {
            printThisBar += plus;
          }
        }
        plus = '';
        printThisBar += '</div>';
        document.write(printThisBar);
        printThisBar = '';
        //SHADOW
        if(oHandle.shadowLightSource != '') {
          //NW
          //  Y shadow +shadowOffset
          //  X Shadow +shadowOffset
          //NE
          //  Y shadow +shadowOffset
          //  X shadow -shadowOffset
          //SW
          //  Y shadow -shadowOffset
          //  X shadow +shadowOffset
          //SE
          //  Y shadow -shadowOffset
          //  X shadow -shadowOffset
          var shadowLeft;
          var shadowTop;
          var shadowLength;
          var shadowOffset = 3;
          if(oHandle.shadowLightSource == 'NW') {
            shadowTop = parseInt(oHandle.yAxLength - barVal[1]) + shadowOffset;
            shadowLength = (parseInt(barVal[1])-parseInt(shadowOffset));
            shadowLeft = (oHandle.gLPO + oHandle.yAxWidth + ( (barSpacing * iterate) - barVal[2] )) + shadowOffset + 1;
          }
          else if(oHandle.shadowLightSource == 'NE') {
            shadowTop = parseInt(oHandle.yAxLength - barVal[1]) + shadowOffset;
            shadowLength = (parseInt(barVal[1])-parseInt(shadowOffset));
            shadowLeft = (oHandle.gLPO + oHandle.yAxWidth + ( (barSpacing * iterate) - barVal[2] )) - shadowOffset + 1;
          }
          else if(oHandle.shadowLightSource == 'SW') {
            shadowTop = parseInt(oHandle.yAxLength - barVal[1]) - shadowOffset;
            shadowLength = (parseInt(barVal[1])+parseInt(shadowOffset));
            shadowLeft = (oHandle.gLPO + oHandle.yAxWidth + ( (barSpacing * iterate) - barVal[2] )) + shadowOffset + 1;
          }
          else if(oHandle.shadowLightSource == 'SE') {
            shadowTop = parseInt(oHandle.yAxLength - barVal[1]) - shadowOffset;
            shadowLength = (parseInt(barVal[1])+parseInt(shadowOffset));
            shadowLeft = (oHandle.gLPO + oHandle.yAxWidth + ( (barSpacing * iterate) - barVal[2] )) - shadowOffset + 1;
          }
          if(shadowLength > oHandle.graphMaxValue) {
            shadowLength = oHandle.graphMaxValue;
            shadowTop = parseInt(oHandle.yAxLength - barLength);
          }
          var printThisShadow = '<div id="'+oHandle.prefixJSGraphYBarShadow+barVal[0].toString()+'s" style="position:absolute;z-index:9;background-color:#C0C0C0;font-size:1px;';
          printThisShadow += 'top:'+shadowTop+'px;';
          printThisShadow += 'left:'+shadowLeft+'px;';
          printThisShadow += 'width:'+barVal[2]+'px;';
          printThisShadow += 'height:'+shadowLength+'px;"></div>';
          document.write(printThisShadow);
          printThisShadow = '';
        }
      }
    }
  } else {
    //Horizontal bars
    var barSpacing = parseInt(oHandle.yAxLength / (oHandle.bars.length + 1));
    var iterate = 0;
    var barAndShadowLeft = parseInt(oHandle.gLPO+oHandle.yAxWidth);
    var bari = 0;
    for(bari=0;bari<=(oHandle.bars.length-1);bari++) {
      if(oHandle.bars[bari]) {
        iterate++;
        var barVal = new Array;
        barVal = oHandle.bars[bari].split(':');
        //barVal[0] = uniqueID
        //barVal[1] = units
        //barVal[2] = width
        //barVal[3] = colour
        //BAR

        var plus;
        var barLength = barVal[1];
        if(barLength > oHandle.graphMaxValue) {
          barLength = oHandle.graphMaxValue;
          plus = '<font style="font-size:9pt;font-weight:bold;">+</font>';
        }
        var printThisBar = '<div id="'+oHandle.prefixJSGraphXBar+barVal[0].toString()+'" style="position:absolute;text-align:right;font-size:1px;border-top:1px outset #F5F5F5;border-right:1px outset #F5F5F5;z-index:10;padding: 0px 0px 0px 0px;';
        printThisBar += 'top:'+(barSpacing * iterate)+'px;';
        printThisBar += 'left:'+barAndShadowLeft+'px;';
        printThisBar += 'width:'+barLength+'px;';
        printThisBar += 'height:'+barVal[2]+'px;';
        printThisBar += 'background-color:'+barVal[3]+';">';
        if(oHandle.usePlusMarker == true) {
          if(plus) {
            printThisBar += plus;
          }
        }
        plus = '';
        printThisBar += '</div>';
        document.write(printThisBar);
        printThisBar = '';
        if(oHandle.shadowLightSource != '') {
          //NW
          //  Y shadow +shadowOffset
          //  X Shadow +shadowOffset
          //NE
          //  Y shadow +shadowOffset
          //  X shadow -shadowOffset
          //SW
          //  Y shadow -shadowOffset
          //  X shadow +shadowOffset
          //SE
          //  Y shadow -shadowOffset
          //  X shadow -shadowOffset
          var shadowTop;
          var shadowLength;
          var shadowOffset = 3;
          if(oHandle.shadowLightSource == 'NW') {
            shadowTop = parseInt((barSpacing * iterate) + shadowOffset);
            shadowLength = parseInt(barLength) + shadowOffset;
          }
          else if(oHandle.shadowLightSource == 'NE') {
            shadowTop = parseInt((barSpacing * iterate) + shadowOffset);
            shadowLength = parseInt(barLength) - shadowOffset;
          }
          else if(oHandle.shadowLightSource == 'SW') {
            shadowTop = parseInt((barSpacing * iterate) - shadowOffset);
            shadowLength = parseInt(barLength) + shadowOffset;
          }
          else if(oHandle.shadowLightSource == 'SE') {
            shadowTop = parseInt((barSpacing * iterate) - shadowOffset);
            shadowLength = parseInt(barLength) - shadowOffset;
          }
          if(shadowLength > oHandle.graphMaxValue) {
            shadowLength = oHandle.graphMaxValue;
          }
          var printThisShadow = '<div id="'+oHandle.prefixJSGraphXBarShadow+barVal[0].toString()+'s" style="position:absolute;z-index:9;background-color:#C0C0C0;font-size:1px;';
          printThisShadow += 'top:'+shadowTop+'px;';
          printThisShadow += 'left:'+barAndShadowLeft+'px;';
          printThisShadow += 'width:'+shadowLength+'px;';
          printThisShadow += 'height:'+barVal[2]+'px;"></div>';
          document.write(printThisShadow);
          printThisShadow = '';
        }
      }
    }
  }
}

function _isNumber(a) {
  return typeof parseFloat(a) == 'number' && isFinite(a);
}

function _get_max_plots(objArray) {
  var lastNum = 0;
  var plotCounter = 0;
  var highestPlotCount = 0;
  for(var oi = 0; oi<objArray.length;oi++) {
    var objArrVal = objArray[oi].split(':');
    if(objArrVal[0] != lastNum) {
      lastNum = objArrVal[0];
      highestPlotCount = plotCounter;
      plotCounter = 0;
    } else {
      plotCounter++;
    }
  }
  if(plotCounter > highestPlotCount) {
    return plotCounter;
  } else {
    return highestPlotCount;
  }
}

function _make_Circle_Div(l,t,w,h,dc) {
  //For some reason I haven't figured out yet,
  //making this part of the section of the code
  //that generates the circle points causes
  //portions of the circle not to be rendered.
  //Go figure.
  var crccolor = dc;
  if(!crccolor) {
    crccolor = this.pieDivColour;
  }
  var htm = '<div style="position:absolute;'+
		'left:' + l + 'px;'+
		'top:' + t + 'px;'+
		'width:' + w + 'px;'+
		'height:' + h + 'px;'+
		'z-index:6;'+
		'clip:rect(0,'+w+'px,'+h+'px,0);'+
		'background-color:' + crccolor + ';"><\/div>';
  document.write(htm);
  htm = '';
}

function _draw_line(oHandle, linePensize, lineColour, varX1, varY1, varX2,varY2) {
  var wider = false;
  var taller = false;
  var deg45 = false;
  var str8line = false;
  var x_abs = 0;
  var y_abs = 0;

  oHandle.barcounter++;
  var plotLineId = 'plotline'+oHandle.id.toString()+'_'+oHandle.barcounter;

  if(varX1 > varX2) {
    x_abs = varX1 - varX2;
  } else {
    x_abs = varX2 - varX1;
  }
  x_abs += 1;
  if(varY1 > varY2) {
    y_abs = varY1 - varY2;
  } else {
    y_abs = varY2 - varY1;
  }
  y_abs += 1;
  if(x_abs == y_abs) {
    deg45 = true;
  }
  else if(x_abs > y_abs) {
    wider = true;
    if(varY1 == varY2) {
      str8line = true;
    }
  }
  else {
    taller = true;
    if(varX1 == varX2) {
      str8line = true;
    }
  }

  if(deg45) {
    var leftToRight = (varX2 < varX1) ? false : true;
    var topToBottom = (varY2 < varY1) ? false : true;
    for(var loopi=0;loopi<x_abs;loopi++) {
      var setLeft; var setTop;
      if(leftToRight) {
        setLeft = parseInt(varX1) + loopi;
      } else {
        setLeft = parseInt(varX1) - loopi;
      }
      if(topToBottom) {
        setTop = parseInt(varY1) + loopi;
      } else {
        setTop = parseInt(varY1) - loopi;
      }
      document.write('<div id="'+plotLineId+loopi.toString()+'" style="position:absolute;left:'+setLeft+'px;top:'+setTop+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:1px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
    }
  }
  else if(wider) {
    if(str8line) {
      var setLeft;
      if(varX1 > varX2) {
        setLeft = varX2;
      } else {
        setLeft = varX1;
      }
      document.write('<div id="'+plotLineId+'" style="position:absolute;left:'+setLeft+'px;top:'+varY1+'px;background-color:'+lineColour+';width:'+x_abs+'px;height:'+linePensize+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
    } else {
      var leftToRight = (varX2 < varX1) ? false : true;
      var topToBottom = (varY2 < varY1) ? false : true;
      var x_last;
      var y_last;
      var segment_last;
      if(leftToRight) {
        var n = new jsnumbers();
        n.value(x_abs / y_abs);
        var d = 0;
        var baseX = varX1;
        var baseY = varY1;
        for(var y_repeat=0;y_repeat<=y_abs;y_repeat++) {
          var segment_width = n.integer();
          d += parseInt(n.Xnth(1));
          if(d>=10) {
            segment_width += 1;
            d -= 10;
          }
          document.write('<div id="'+plotLineId+y_repeat.toString()+'" style="position:absolute;left:'+baseX+'px;top:'+baseY+'px;background-color:'+lineColour+';width:'+segment_width+'px;height:'+linePensize+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          baseX += segment_width;
          if(baseX > varX2) {
            baseX = varX2;
          }
          if(topToBottom) {
            baseY += 1;
            if(baseY > varY2) {
              baseY = varY2;
            }
          } else {
            baseY -= 1;
            if(baseY < varY2) {
              baseY = varY2;
            }
          }
          x_last = baseX;
          y_last = baseY;
          segment_last = segment_width;
        }
        var extraX = x_last + segment_last;
        if(extraX < varX2) {
          var extraWidth = varX2 - extraX;
          if(extraWidth > 0) {
            document.write('<div id="'+plotLineId+extraX.toString()+'" style="position:absolute;left:'+extraX+'px;top:'+y_last+'px;background-color:'+lineColour+';width:'+extraWidth+'px;height:'+linePensize+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }
        }
      } else {
        var n = new jsnumbers();
        n.value(x_abs / y_abs);
        var d = 0;
        var baseX = varX1;
        var baseY = varY1;
        for(var y_repeat=0;y_repeat<=y_abs;y_repeat++) {
          var segment_width = n.integer();
          d += parseInt(n.Xnth(1));
          if(d>=10) {
            segment_width += 1;
            d -= 10;
          }
          baseX -= segment_width;
          if(baseX < varX2) {
            baseX = varX2;
          }
          document.write('<div id="'+plotLineId+y_repeat.toString()+'" style="position:absolute;left:'+baseX+'px;top:'+baseY+'px;background-color:'+lineColour+';width:'+segment_width+'px;height:'+linePensize+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          if(topToBottom) {
            baseY += 1;
            if(baseY > varY2) {
              baseY = varY2;
            }
          } else {
            baseY -= 1;
            if(baseY < varY2) {
              baseY = varY2;
            }
          }
          x_last = baseX;
          y_last = baseY;
          segment_last = segment_width;
        }
        var extraX = x_last - segment_last;
        if(extraX < varX2) {
          var extraWidth = varX2 - extraX;
          if(extraWidth > 0) {
            document.write('<div id="'+plotLineId+extraX.toString()+'" style="position:absolute;left:'+extraX+'px;top:'+y_last+'px;background-color:'+lineColour+';width:'+extraWidth+'px;height:'+linePensize+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }
        }
      }
    }
  } else {
    //taller
    if(str8line) {
      var setTop;
      if(varY1 > varY2) {
        setTop = varY2;
      } else {
        setTop = varY1;
      }
      document.write('<div id="'+plotLineId+'" style="position:absolute;left:'+varX1+'px;top:'+setTop+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+y_abs+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
    } else {
      var leftToRight = (varX2 < varX1) ? false : true;
      var topToBottom = (varY2 < varY1) ? false : true;
      if(leftToRight) {
        var n = new jsnumbers();
        n.value(y_abs / x_abs);
        var d = 0;
        var baseY = varY1;
        var baseX = varX1;
        var x_last;
        var y_last;
        var segment_last;
        for(var x_repeat=0;x_repeat<=x_abs;x_repeat++) {
          var segment_height = n.integer();
          d += parseInt(n.Xnth(1));
          if(d>=10) {
            segment_height += 1;
            d -= 10;
          }
          var setTop;
          if(topToBottom) {
            setTop = baseY;
            if( (baseY + segment_height) > varY2) {
              segment_height = varY2 - baseY;
            }
          } else {
            setTop = baseY - segment_height;
            if( setTop < varY2) {
              segment_height = y_last - varY2;
              setTop = varY2;
            }
          }
          document.write('<div id="'+plotLineId+x_repeat.toString()+'" style="position:absolute;left:'+baseX+'px;top:'+setTop+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+segment_height+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          x_last = baseX;
          y_last = baseY;
          segment_last = segment_height;
          baseX += 1;
          if(baseX > varX2) {
            baseX = varX2;
          }
          if(topToBottom) {
            baseY += segment_height;
            if(baseY > varY2) {
              baseY = varY2;
            }
          } else {
            baseY -= segment_height;
            if(baseY < varY2) {
              baseY = varY2;
            }
          }
        }
        if(topToBottom) {
          var extraY = y_last + segment_last;
          if(extraY < varY2) {
            var extraHeight = varY2 - extraY;
            document.write('<div id="'+plotLineId+extraY.toString()+'" style="position:absolute;left:'+x_last+'px;top:'+extraY+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+extraHeight+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }

        } else {
          var extraY = y_last - segment_last;
          if(extraY > varY1) {
            var extraHeight = extraY - varY1;
            document.write('<div id="'+plotLineId+extraY.toString()+'" style="position:absolute;left:'+x_last+'px;top:'+extraY+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+extraHeight+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }
        }
      } else {
        //Right to left
        var n = new jsnumbers();
        n.value(y_abs / x_abs);
        var d = 0;
        var baseY = varY1;
        var baseX = varX1;
        if(n.Xnth(1) > 0) {
          if(topToBottom) {
            baseY++;
          } else {
            baseY--;
          }
        }
        var x_last;
        var y_last;
        var segment_last;
        for(var x_repeat=x_abs;x_repeat>=0;x_repeat--) {
          var segment_height = n.integer();
          d += parseInt(n.Xnth(1));
          if(d>=10) {
            segment_height += 1;
            d -= 10;
          }
          var setTop;
          if(topToBottom) {
            setTop = baseY;
            if( (baseY + segment_height) > varY2) {
              segment_height = varY2 - baseY;
            }
          } else {
            setTop = baseY - segment_height;
            if( setTop < varY2) {
              segment_height = y_last - varY2;
              setTop = varY2;
            }
          }
          document.write('<div id="'+plotLineId+x_repeat.toString()+'" style="position:absolute;left:'+baseX+'px;top:'+setTop+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+segment_height+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          x_last = baseX;
          y_last = baseY;
          segment_last = segment_height;
          baseX -= 1;
          if(baseX < varX2) {
            baseX = varX2;
          }
          if(topToBottom) {
            baseY += segment_height;
            if(baseY > varY2) {
              baseY = varY2;
            }
          } else {
            baseY -= segment_height;
            if(baseY < varY2) {
              baseY = varY2;
            }
          }
        }
        if(topToBottom) {
          var extraY = y_last + segment_last;
          if(extraY < varY2) {
            var extraHeight = varY2 - extraY;
            document.write('<div id="'+plotLineId+extraY.toString()+'" style="position:absolute;left:'+x_last+'px;top:'+extraY+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+extraHeight+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }
        } else {
          var extraY = y_last - segment_last;
          if(extraY > varY1) {
            var extraHeight = extraY - varY1;
            document.write('<div id="'+plotLineId+extraY.toString()+'" style="position:absolute;left:'+x_last+'px;top:'+extraY+'px;background-color:'+lineColour+';width:'+linePensize+'px;height:'+extraHeight+'px;font-size:1px;padding:0px;margin:0px;z-index:100;"></div>');
          }
        }
      }
    }
  }
}

function jsnumbers (nV) {
  this.numberValue = parseFloat(nV);
  this.cleaned = 0;
  this.integer = function() {
    this.clean();
    return parseInt(this.numberValue);
  }
  this.dollar = function() {
    this.clean();
    return this.decimal(2);
  }
  this.decimal = function () {
    this.clean();
    if(arguments[0]) {
      var nv = new String(this.numberValue);
      if(nv.indexOf('.') >= 0) {
        var nparts = new Array();
        nparts = nv.split('.');
        if(nparts[1].length > arguments[0]) {
          var rounder = nparts[1].charAt(arguments[0]);
          nparts[1] = nparts[1].substr(0, arguments[0]);
          if(rounder >= 5) {
            nparts[1] = parseInt(nparts[1]) + 1;
          }
          var nn = parseInt(this.numberValue);
          return nn.toString()+'.'+nparts[1].toString();
        } else {
          return this.padding(arguments[0]);
        }
      } else {
        return this.integer();
      }
    } else {
      return this.integer();
    }
  }
  this.padding = function() {
    this.clean();
    if(arguments[0]) {
      var nv = new String(this.numberValue);
      if(nv.indexOf('.') >= 0) {
        var nparts = new Array();
        nparts = nv.split('.');
        if(nparts[1].length<arguments[0]) {
          nparts[1] += this.repeat('0', (parseInt(arguments[0]) - nparts[1].length));
        }
        return nparts[0]+'.'+nparts[1];
      } else {
        return n+'.'+this.repeat('0', arguments[0]);
      }
    }
  }
  this.paddingLeft = function() {
    this.clean();
    var plChar;
    if(!arguments[1]) {
      plChar = ' ';
    } else {
      plChar = arguments[1];
    }
    if(arguments[0]) {
      var nv = new String(this.numberValue);
      if(nv.indexOf('.') >= 0) {
        var nparts = new Array();
        nparts = nv.split('.');
        if(nparts[0].length<arguments[0]) {
          nparts[0] = this.repeat(plChar, (parseInt(arguments[0]) - nparts[0].length)) + nparts[0];
        }
        return nparts[0]+'.'+nparts[1];
      } else {
        return this.repeat(plChar, (parseInt(arguments[0]) - n.length))+n;
      }
    }
  }
  this.clean = function() {
    if(!this.cleaned) {
      this.cleaned = 1;
      var ival = this.numberValue;
      var oval = '';
      var oi = 0;
      for(oi=0;oi<ival.length;oi++) {
        if( !isNaN(ival.charAt(oi)) ) {
          oval += ival.substr(oi, 1);
        } else {
          if(ival.charAt(oi)=='.') {
            oval += '.';
          }
        }
      }
      this.numberValue = oval;
    }
  }
  this.value = function() {
    if(arguments[0]) {
      this.numberValue = arguments[0];
      if(this.isNumber(this.numberValue)) {
        this.numberValue = this.numberValue.toString();
      }
      this.cleaned = 0;
      this.clean();
    } else {
      return this.numberValue;
    }
  }
  this.repeat = function() {
    var rc = arguments[0];
    if(!rc) {
      rc = ' ';
    }
    if(arguments[1]) {
      var rv = '';
      var ri = 0;
      for(ri=1;ri<=parseInt(arguments[1]);ri++) {
        rv += rc.toString();
      }
      return rv;
    } else {
      return rc;
    }
  }
  this.isNumber = function() {
    return typeof arguments[0] == 'number' && isFinite(arguments[0]);
  }
  this.isInt = function() {
    var la_mod = parseFloat(this.decimal(2)) % 1;
    if(la_mod == 0) {
      return true;
    } else {
      return false;
    }
  }
  this.Xnth = function() {
    var nth = arguments[0];
    if(!nth) {nth = 1;}
    var nv = new String(this.numberValue);
    if(nv.indexOf('.') >= 0) {
      var nparts = new Array();
      nparts = nv.split('.');
      if(nparts[1].length > nth) {
        var rounder = nparts[1].charAt(nth);
        nparts[1] = nparts[1].substr(0, nth);
        if(rounder >= 5) {
          nparts[1] = parseInt(nparts[1]) + 1;
        }
        return nparts[1].toString();
      } else {
        return nparts[1].toString();
      }
    } else {
      return '0';
    }
  }

  //Called once with object instantiated
  if(this.isNumber(this.numberValue)) {
    this.numberValue = this.numberValue.toString();
  }
}
