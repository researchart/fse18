#!bin/sh
# {"to":"server1.conn1.child1/tab1","type":"navigateTo","url":"file:///home/daniel/Documents/masterthesis/test/bugreports/ff-setBreakpoint-empty-lines/test.html"}

{"to":"root","type":"listTabs"}
{"to":"server1.conn2.child1/tab1","type":"attach"} # attach to tab with test.html, get its thread actor
{"to":"server1.conn2.child1/28","type":"attach"}   # attach to thread actor, pauses it
{"to":"server1.conn2.child1/tab1","type":"reload"} # I don't know why I need this, but otherwise the sources array is empty
{"to":"server1.conn2.child1/28","type":"sources"}  # get test.js source actor
{"to":"server1.conn2.child1/30","type":"setBreakpoint","location":{"line":4},"noSliding":false}
