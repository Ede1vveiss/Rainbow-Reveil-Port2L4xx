/*
 * canvas.h
 *
 *  Created on: 9 oct. 2023
 *      Author: alyvasseur
 *      Modif : Eden Wegger Funderskov
 */

#ifndef INC_CANVAS_H_
#define INC_CANVAS_H_

#include "stm32l4xx_hal.h"
#include "includes.h"

// Définition de la structure Pixel
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} Pixel;

// Définition de la structure Canvas
typedef struct {
    int numCols;
    int numRows;
    Pixel* pixels;
} Canvas;



// Prototypes de fonctions
void setCanvasColor(Canvas* canvas, Pixel encre);
void sendCanvas(Canvas* canvas);
Pixel* getPixel(Canvas* canvas, int x, int y);
void colorDiagonal(Canvas* canvas, Pixel encre, uint8_t diagSum);
void drawRectangle(Canvas* canvas, int W, int H, int X, int Y, Pixel BorderInk, Pixel FillInk);
void displayBCD(Canvas* canvas, int X, int Y, int BCD, int NbDeBitAffiches);
void drawImage(ImageData* imageData, int frame, int x, int y, Canvas* canvas);

void drawIndexedImage(IndexedImageData* indexedImage, int frame, int x, int y, Canvas* canvas);
void convertIndexedToRGBA(IndexedImageData* indexedImage, int frame, ImageData* rgbaImage);

#endif /* INC_CANVAS_H_ */
