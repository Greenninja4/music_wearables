
// Recieving OSC messages
(
OSCdef.new(\recv, {
	arg msg;
	msg.postln;
}, '/testAddr');
)

thisProcess.openPorts; // list all open ports

NetAddr.localAddr    // retrieve the current IP and port

// sending

n = NetAddr.new("192.168.2.5", 57120);
n.sendMsg('/testAddr', rrand(-180.0,180.0), rrand(-180.0,180.0), rrand(-180.0,180.0));

OSCdef.freeAll        // Free all OSC objects


(
MIDIClient.init;
MIDIIn.connectAll;
)

MIDIFunc.trace(true);
MIDIFunc.trace(false);

(
~mesg = [0,0,0];

MIDIdef.cc(\ccTest, {
	arg val, num, chan, src;
	if(
		num == 1,
		{
			~mesg[0] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2]);
		}
	);
	if(
		num == 2,
		{
			~mesg[1] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2]);
		}
	);
	if(
		num == 3,
		{
			~mesg[2] = (val * (360/127)) - 180;
			n.sendMsg('/testAddr', ~mesg[0], ~mesg[1], ~mesg[2]);
		}
	);
});
)

MIDIdef(\ccTest).free;

~mesg.postln