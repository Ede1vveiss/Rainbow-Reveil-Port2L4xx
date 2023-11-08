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
    uint16_t totalheight;   // Hauteur de toutes les images
    uint16_t height;		// Hauteur de l'image
    uint8_t* data;      	// Tableau de données de l'image (RGBA ou autre)
} ImageData;

typedef struct {
	uint8_t LineLen;		// nombre de uint8 sur 1 ligne
    uint16_t width;     // Largeur de l'image
    uint16_t totalheight;   // Hauteur de toutes les images
    uint16_t height;    // Hauteur de l'image
    uint8_t* data;      // Tableau de données de l'image (Indexed 2 bit)
} IndexedImageData;

extern IndexedImageData BadApple;
extern ImageData NotPickleRick;

#endif /* LIBRARY_INC_IMAGE_DATA_H_ */
