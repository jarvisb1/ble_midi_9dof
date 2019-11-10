#ifndef _NOTES_H_
#define _NOTES_H_

#include <Arduino.h>
#include "pitchToNote.h"

#define NUM_NOTES (12)

//C Major
const byte note_pitches_roll[NUM_NOTES] = {pitchC3, pitchD3, pitchE3, pitchF3, pitchG3, pitchA3, pitchB3, pitchC4, pitchD4, pitchE4, pitchF4, pitchG4};
const byte note_pitches_pitch[NUM_NOTES] = {pitchA4, pitchB4, pitchC5, pitchD5, pitchE5, pitchF5, pitchG5, pitchA5, pitchB5, pitchC6, pitchD6, pitchE6};
const byte note_pitches_heading[NUM_NOTES] = {pitchF6, pitchG6, pitchA6, pitchB6, pitchC7, pitchD7, pitchE7, pitchF7, pitchG7, pitchA7, pitchB7, pitchC8};

//Major pentatonic, white-key transposed
//const byte note_pitches[NUM_NOTES] = {pitchC4, pitchD4, pitchE4, pitchG4, pitchA4, pitchC5, pitchD5, pitchE5, pitchG5, pitchA5, pitchC6, pitchD6};

//Egyptian, suspended, white-key transposed
//const byte note_pitches[NUM_NOTES] = {pitchG4, pitchA4, pitchC5, pitchD5, pitchF5, pitchG5, pitchA5, pitchC6, pitchD6, pitchF6, pitchG6, pitchA6};


#endif _NOTES_H_