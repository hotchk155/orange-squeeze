// midi_test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

char notes[16];
char vels[16];
DWORD time[16] = { 0 };

void CALLBACK MidiInProc(
	HMIDIIN hMidiIn,
	UINT wMsg,
	DWORD dwInstance,
	DWORD dwParam1,
	DWORD dwParam2
)
{
	if (wMsg != 0x3c3)
		return;
	if ((dwParam1 & 0xF0) == 0x90) {
		byte chan = (byte)(dwParam1 & 0x0F);
		byte note = (byte)(dwParam1 >> 8);
		byte vel = (byte)(dwParam1 >> 16);

		printf(".");
		time[chan] = GetTickCount();

		if (!vel) { // note off
			if (!vels[chan]) { // note not on
				printf("ch %d unexpected note off %d\n", chan, note);
			}
			else  if (notes[chan] != note) { // make sure its the right note
				printf("ch %d note %d mismatch\n", chan, notes[chan]);
			}
			vels[chan] = 0;
		}
		else { // note on
			if (vels[chan]) { // note already on
				printf("ch %d unexpected note on %d when %d playing\n", chan, note, notes[chan]);
			}
			else if ((notes[chan] + 1 != note) && !(notes[chan] == 127 && note == 1)) {
				printf("ch %d unexpected note on %d when last note was %d\n", chan, note, notes[chan]);
			}
			notes[chan] = note;
			vels[chan] = vel;
		}

	}
}

//////////////////////////////////////////////////////////
// MIDI IN DEVICE
HMIDIIN OpenDevice(const WCHAR *szDevice)
{

	UINT uiDevices = midiInGetNumDevs();
	for (int iDevice = 0; iDevice < (int)uiDevices; ++iDevice)
	{
		MIDIINCAPS stMIC = { 0 };
		if (MMSYSERR_NOERROR == midiInGetDevCaps(iDevice, &stMIC, sizeof(stMIC)))
		{
			printf("%S\n", stMIC.szPname);
			if (0 == wcscmp(stMIC.szPname, szDevice))
			{
				HMIDIIN hMidiIn = NULL;
				if (MMSYSERR_NOERROR == midiInOpen(
					&hMidiIn,
					iDevice,
					(DWORD)MidiInProc, //dwCallback,          
					0, //dwCallbackInstance,  
					CALLBACK_FUNCTION //dwFlags              
				))
				{
					if (MMSYSERR_NOERROR == midiInStart(hMidiIn))
					{
						printf("OK\n");
						return hMidiIn;
					}
					midiInClose(hMidiIn);
				}
			}
		}
	}
	printf("FAILED\n");
	return FALSE;
}


int main()
{
	HMIDIIN hMidiIn = OpenDevice(L"Delta 1010LT MIDI");
	while (!_kbhit()) {
		DWORD t = GetTickCount() - 400;
		for (int i = 0; i < 16; ++i) {
			if (time[i] > 0 && time[i] < t) {
				printf("channel %d input stopped\n", i);
				time[i] = 0;
			}
		}
	}
	_getch();
	if(hMidiIn) {
		midiInReset(hMidiIn);
		midiInClose(hMidiIn);
	}
    return 0;
}

