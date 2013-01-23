#ifndef GOBANPROTO_H
#define GOBANPROTO_H

// Command numbers

enum msgs {
  ACK = 0,  // Ack
  NAK, // Non Ack
  GETLEDS,  // Get LEDs
  SETLEDS,  // Set LEDs
  GETSTATE,  // Get state
  NEWSTATE   // New state
};

#endif
