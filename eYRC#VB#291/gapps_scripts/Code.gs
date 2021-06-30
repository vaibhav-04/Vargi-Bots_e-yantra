function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }

    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;  
  }
  updateDashboard();
  if(e.parameter["id"] == "OrdersDispatched") {
    dispatchAlert();
  }
  else if(e.parameter["id"] == "OrdersShipped") {
    shipAlert();
  }
  return ContentService.createTextOutput('success');
}

function updateDashboard() {
  var sheets =  SpreadsheetApp.getActive();
  var orders = sheets.getSheetByName("IncomingOrders");
  var dispatched = sheets.getSheetByName("OrdersDispatched");
  var shipped = sheets.getSheetByName("OrdersShipped");
  var dashboard = sheets.getSheetByName("Dashboard");
  var dataOrders = orders.getDataRange().getValues();
  var dataDispatched = dispatched.getDataRange().getValues();
  var dataShipped = shipped.getDataRange().getValues();
  var dataDashboard = dashboard.getDataRange().getValues(); 
  var flag = 0;
  for(var i = 1;i<dataOrders.length;i++)  {
    flag = 0;
    for(var j = 1;j<dataDashboard.length;j++) {
      if(dataOrders[i][3] == dataDashboard[j][0])  {
        flag = 1;
        break;
      } 
    }
    if(flag == 0) {
      appendData(dashboard, dataOrders[i]);
      // Logger.log(dataOrders[i]);
    }
  }
  dataDashboard = dashboard.getDataRange().getValues();
  flag = 0;
  for(var i = 1;i<dataDispatched.length;i++)  {
    flag = 0;
    var j = 1;
    for(j;j<dataDashboard.length;j++) {
      if(dataDispatched[i][3] == dataDashboard[j][0] && dataDispatched[i][9] != dataDashboard[j][6])  {
        flag = 1;
        break;
      } 
    }
    if(flag == 1) {
      updateData(dashboard, j, 6, dataDispatched[i][9]);
      updateData(dashboard, j, 9, dataDispatched[i][10]);
    }
  }
  dataDashboard = dashboard.getDataRange().getValues();

  flag = 0;
  for(var i = 1;i<dataShipped.length;i++)  {
    flag = 0;
    var j = 1;
    for(j;j<dataDashboard.length;j++) {
      if(dataShipped[i][3] == dataDashboard[j][0] && dataShipped[i][9] != dataDashboard[j][7])  {
        flag = 1;
        break;
      } 
    }
    if(flag == 1) {
      updateData(dashboard, j, 7, dataShipped[i][9]);
      updateData(dashboard, j, 10, dataShipped[i][10]);
      
    }
  }
  dataDashboard = dashboard.getDataRange().getValues();
  for(var i=1;i<dataDashboard.length;i++) {
    if(dataDashboard[i][10] != "" && dataDashboard[i][11] == "")  {
      timeTaken(dashboard, i, dataDashboard[i]);
    }
  }
}

function appendData(sheet, data)  {
  // Logger.log(data);
  var pushData = [];
  pushData.push(data[3]);
  pushData.push(data[5]);
  pushData.push(data[6]);
  pushData.push(data[8]);
  pushData.push(data[9]);
  pushData.push(data[10]);
  pushData.push('');
  pushData.push('');
  pushData.push(data[4]);
  pushData.push('');
  pushData.push('');
  sheet.appendRow(pushData);
}

function updateData(sheet, row, col, data)  {
  var pushData = data;
  var cell = sheet.getRange('a1');
  cell.offset(row, col).setValue(pushData);
}

function timeTaken(sheet, row, data)  {

  var dateShipped = new Date(data[10]);
  var dateOrdered = new Date(data[8]);
  var timeShipped = dateShipped.getTime();
  var timeOrdered = dateOrdered.getTime();
  var pushData = Math.floor(timeShipped-timeOrdered)/(1000);
  var cell = sheet.getRange('a1');
  cell.offset(row, 11).setValue(pushData);
}

function dispatchAlert()  {
  var sheets =  SpreadsheetApp.getActive();
  var dispatched = sheets.getSheetByName("OrdersDispatched");
  var data = dispatched.getDataRange().getValues();
  var lastRow = dispatched.getLastRow();
  var dispatchedDate = new Date(data[lastRow-1][10]);
  var formatChangeDispatchedDate = Utilities.formatDate(dispatchedDate, "GMT+5:30", "MMMM dd yyyy HH:mm:ss");
  var to = "eyrc.vb.0291@gmail.com";   //write your email id here
  var message = "Hello!\n\nYour order has been dispatched. Contact us if you have any query, we are here to help you.\n\nORDER SUMMARY:\n\nOrder Number: " + data[lastRow-1][3] + " \nItem: "+data[lastRow-1][5]+"\nQuantity: "+data[lastRow-1][7]+"\nDispatch Date and Time: "+formatChangeDispatchedDate+"\nCity: "+data[lastRow-1][4]+"\nCost: "+data[lastRow-1][8] ; 

    MailApp.sendEmail(to, " Your Order is Dispatched! ", message);

}

function shipAlert()  {
  var sheets =  SpreadsheetApp.getActive();
  var shipped = sheets.getSheetByName("OrdersShipped");
  var data = shipped.getDataRange().getValues();
  var lastRow = shipped.getLastRow();
  var shippedDate = new Date(data[lastRow-1][10]);
  var formatChangeShippedDate = Utilities.formatDate(shippedDate, "GMT+5:30", "MMMM dd yyyy HH:mm:ss");
  var estimatedDate = new Date(data[lastRow-1][11]);
  var formatChangeEstimatedDate = Utilities.formatDate(estimatedDate, "GMT+5:30", "MMMM dd yyyy");
  // Logger.log(formatChangeShippedDate);
  // Logger.log(formatChangeEstimatedDate);
  
  // Logger.log(dataShipped[lastRow-1]);
  var to = "eyrc.vb.0291@gmail.com";   //write your email id here
  var message = "Hello!\n\nYour order has been shipped. Contact us if you have any query, we are here to help you.\n\nORDER SUMMARY:\n\nOrder Number: " + data[lastRow-1][3] + " \nItem: "+data[lastRow-1][5]+"\nQuantity: "+data[lastRow-1][7]+"\nShipped Date and Time: "+formatChangeShippedDate+"\nCity: "+data[lastRow-1][4]+"\nCost: "+data[lastRow-1][8]+"\nEstimated Time of Delivery: "+formatChangeEstimatedDate; 

    MailApp.sendEmail(to, " Your Order is Shipped! ", message);

}

function test() {
  Logger.log(MailApp.getRemainingDailyQuota());
}