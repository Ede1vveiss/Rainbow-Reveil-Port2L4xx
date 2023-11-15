/*
 * PlaceHolder_data.c
 *
 *  Created on: 14 nov. 2023
 *      Author: Eden W. Funderskov
 */

#include "includes.h"

/*
 * Pour le format de couleurs indexé, BitMask varie selon le nombre de bits choisi pour l'indexé
 * 		- 1 bit  	BitMask = 0x01
 * 		- 2 bits  	Bitmask = 0x03
 * 		- 4 bits 	Bitmask = 0x0f
 * 		- 8 bits 	BitMask = 0xff
 */

IndexedImageData PlaceHolder_data = {
	.BitMask 	= 	0x03,		// Mask pour 2 bits
	.LineLen 	= 	xxx,		// Nombre de uint8 sur une ligne dans le tableau .data
    .width 		=	xxx,        // Largeur d'une image (en pixels)
	.FrameAmount =	xxx,    	// Quantité d'images dans l'animation
    .height 	=	xxx,       	// Hauteur d'une image (en pixels)
	.ColorPalette = (uint8_t[]){
		// insérer la palette de couleurs ici

	},
    .data = (uint8_t[]) {
		// insérer l'emplacement ici

    }
};
