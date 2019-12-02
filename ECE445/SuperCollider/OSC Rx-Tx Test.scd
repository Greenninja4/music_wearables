~btn1_state = false;
// Recieving OSC messages
(
OSCdef.new(\recv, {
	arg msg, x, y;
	//msg.postln;
/*	if((msg[5] != ~btn1_state) && (msg[5]==false),
		{"event1".postln; ~btn1_state = msg[5]},
		{if((msg[5] != ~btn1_state) && (msg[5]==true),
			{"event2".postln; ~btn1_state = msg[5]},{
				nil;
			}
		)}
	);*/
y = case
		{(msg[5] != ~btn1_state) && (msg[5]==false)} {"event1".postln; ~btn1_state = msg[5];}
		{(msg[5] != ~btn1_state) && (msg[5]==true)} {"event2".postln; ~btn1_state = msg[5];}
		{true} {nil};

	x = [msg[5], y];
	x.postln;
	// btn1_state.postln;

}, '/device1');
)

thisProcess.openPorts; // list all open ports

NetAddr.localAddr    // retrieve the current IP and port

// sending

n = NetAddr.new("192.168.2.6", 57120);
n.sendMsg('/testAddr', rrand(-180.0,180.0), rrand(-180.0,180.0), rrand(-180.0,180.0));

OSCdef.freeAll        // Free all OSC objects


(
MIDIClient.init;
MIDIIn.connectAll;
)

MIDIFunc.trace(true);
MIDIFunc.trace(false);

(
~mesg = [0,0,0,0,0,0,0];

MIDIdef.cc(\ccTest, {
	arg val, num, chan, src;
	if(
		num == 1,
		{
			~mesg[0] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
	if(
		num == 2,
		{
			~mesg[1] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
	if(
		num == 3,
		{
			~mesg[2] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
	if(
		num == 5,
		{
			if(val <= 64, {~mesg[3] = 0}, {~mesg[3] = 1});
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
});

MIDIdef.noteOn(\onTest, {
	arg val, num, chan, src;
//	[val, num, chan, src].postln;
	if(
		num == 55,
		{
			~mesg[3] = 1;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
	if(
		num == 57,
		{
			~mesg[4] = 1;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);if(
		num == 48,
		{
			~mesg[5] = 1;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);if(
		num == 50,
		{
			~mesg[6] = 1;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
});

MIDIdef.noteOff(\offTest, {
	arg val, num, chan, src;
//	[val, num, chan, src].postln;
	if(
		num == 55,
		{
			~mesg[3] = 0;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
	if(
		num == 57,
		{
			~mesg[4] = 0;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);if(
		num == 48,
		{
			~mesg[5] = 0;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);if(
		num == 50,
		{
			~mesg[6] = 0;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2], ~mesg[3], ~mesg[4], ~mesg[5], ~mesg[6]);
		}
	);
});
)

MIDIdef.freeAll;
~mesg.postln