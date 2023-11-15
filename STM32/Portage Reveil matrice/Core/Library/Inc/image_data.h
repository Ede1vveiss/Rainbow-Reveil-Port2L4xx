/*
 * image_data.h
 *
 *  Created on: 30 oct. 2023
 *      Author: alyvasseur
 *      Modif : Eden W. Funderskov
 */

#ifndef LIBRARY_INC_BADAPPLE_DATA_H_
#define LIBRARY_INC_BADAPPLE_DATA_H_

#include "includes.h"

typedef struct {
    uint16_t width; 	    // Largeur de l'image
    uint16_t FrameAmount;   // Hauteur de toutes les images
    uint16_t height;		// Hauteur de l'image
    uint8_t* data;      	// Tableau de données de l'image (RGBA ou autre)
} ImageData;

/*
 * Pour l'indexé, BitMask varie selon le nombre de bits
 * 		- 1 bit  	BitMask = 0x01
 * 		- 2 bits  	Bitmask = 0x03
 * 		- 4 bits 	Bitmask = 0x0f
 * 		- 8 bits 	BitMask = 0xff
 *
 */
typedef struct {
	uint8_t BitMask;		// Mask pour n nombre de bits
	uint8_t LineLen;		// nombre de uint8 sur 1 ligne dans le tableau .data
    uint16_t width;     	// Largeur de l'image
    uint16_t FrameAmount;   // Quantité d'images dans l'animation
    uint16_t height;    	// Hauteur de l'image
    uint8_t* ColorPalette;	// Tableau de données des couleurs de l'image
    uint8_t* data;      	// Tableau de données de l'image (Indexés)
} IndexedImageData;

extern IndexedImageData BadApple_1bit;
extern IndexedImageData BadApple_1bit_15fps;
extern IndexedImageData BadApple_2bit;
extern IndexedImageData BadApple_2bit_7dot5fps;
extern IndexedImageData BadApple_4bit;
extern IndexedImageData BadApple_4bit_7dot5fps;
extern ImageData NotPickleRick;
extern IndexedImageData NotPickleRickIndexed;

#endif /* LIBRARY_INC_IMAGE_DATA_H_ */
