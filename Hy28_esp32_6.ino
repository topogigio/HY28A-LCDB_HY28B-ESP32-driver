// Driver code for use ESP32 to drive HY28 LCD display including touch panel
/*******************************************************************************
* Function Name  : Driver HY28A-LCDB / HY28B SPI
* Description    : Driver for LCD, HY28A-LCDB using: ILI9320 for LCD & ADS7843 (XPT2046) for Touch Panel  
*                : Driver for LCD, HY28B using: ILI9325 for LCD & ADS7843 (XPT2046) for Touch Panel  
* 
* Code for       : ESP32 driver porting from Arduino due of a subseguent porting from Raspberry PI
* Author         : Andrea Marotta (marotta.andrea@gmail.com)
*                  Fork project of PJ McCracken (bauderline@gmail.com)
*******************************************************************************/

// 19 - TP_SDO - LCD_SDO
// 18 - TP_SCK - LCD_SCK
// 23 - TP_SDI - LCD_SDI
// 27 - TP_IRQ
// 26 - TP_CS
// 13 - LCD_RST
//  5 - LCD_CS
//
// 3.3v IN
// GND

#include <SPI.h>
#include "FS.h"
#include "SPIFFS.h"
#include "hy28b.h"
#include "fonts.h"

#define FORMAT_SPIFFS_IF_FAILED true

#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18
#define VSPI_SS     5

#define THRESHOLD 2

#define LCD_CS 5
#define LCD_CS_LO digitalWrite(LCD_CS, LOW)
#define LCD_CS_HI digitalWrite(LCD_CS, HIGH)

#define LCD_RESET 13
#define LCD_RESET_LO digitalWrite(LCD_RESET, LOW)
#define LCD_RESET_HI digitalWrite(LCD_RESET, HIGH)

#define TP_CS 26
#define TP_IRQ 27
#define TP_CS_LO digitalWrite(TP_CS, LOW)
#define TP_CS_HI digitalWrite(TP_CS, HIGH)

#define ONE_LED 2

#define DISABLE 0
#define ENABLE 1

#define VERTICAL 0
#define HORIZONTAL 1
//#define LCD_ALIGNMENT VERTICAL
#define LCD_ALIGNMENT HORIZONTAL

#define PREC_LOW 1
#define PREC_MEDIUM 2
#define PREC_HI 3
#define PREC_EXTREME 4

//Color codes
#define RGB565CONVERT(red,green,blue) (unsigned short)(((red>>3)<<11)|((green>>2)<<5)|(blue>>3))

#define RED 0xf800
#define GREEN 0x07e0
#define BLUE 0x001f
#define WHITE 0xffff
#define BLACK 0x0000
#define YELLOW 0xFFE0
#define GREY 0xF7DE
#define GREY2 RGB565CONVERT(0xA0, 0xA0, 0xA0)
#define BLUE2 0x051F
#define MAGENTA 0xF81F
#define CYAN 0x7FFF

#define SPI_START (0x70)           /* Start byte for SPI transfer        */
#define SPI_RD (0x01)              /* WR bit 1 within start              */
#define SPI_WR (0x00)              /* WR bit 0 within start              */
#define SPI_DATA (0x02)            /* RS bit 1 within start byte         */
#define SPI_INDEX (0x00)           /* RS bit 0 within start byte         */

static const int spiClk = 78000000; // 78 MHz
static const int spiClk2 = 2500000; // 2.5 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * vspi2 = NULL;

UTouch myTouch(TP_CS, TP_IRQ);

int orien;
int MAX_X = 240;
int MAX_Y = 320;

struct cfont
{
  uint8_t* font;
  uint8_t x_size;
  uint8_t y_size;
  uint8_t offset;
  uint8_t numchars;
};


/*******************************************************************************
* Function Name  : LCD_WriteIndex
* Description    : LCD write register address
* Input          : - index: register address
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_WriteIndex(unsigned char index) {
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MSBFIRST SPI_MODE0
  LCD_CS_LO;
  vspi->transfer(SPI_START | SPI_WR | SPI_INDEX);
  vspi->transfer(0);
  vspi->transfer(index);  
  LCD_CS_HI;
  vspi->endTransaction();
}


/*******************************************************************************
* Function Name  : LCD_WriteData
* Description    : LCD write register data
* Input          : - data: register data
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_WriteData(unsigned short data) {
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MSBFIRST SPI_MODE0
  LCD_CS_LO;
  vspi->transfer(SPI_START | SPI_WR | SPI_DATA);  
  vspi->transfer((data >> 0x08));  
  vspi->transfer((data & 0xFF));  
  LCD_CS_HI;
  vspi->endTransaction();
}


/*******************************************************************************
* Function Name  : LCD_Write_Data_Start
* Description    : Start of data writing to the LCD controller
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Write_Data_Start(void) {
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MSBFIRST SPI_MODE0
  LCD_CS_LO;
  vspi->transfer(SPI_START | SPI_WR | SPI_DATA);    /* Write : RS = 1, RW = 0       */
  LCD_CS_HI;
  vspi->endTransaction();
}


/*******************************************************************************
* Function Name  : LCD_Write_Data_Only
* Description    : Data writing to the LCD controller
* Input          : - data: data to be written  
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Write_Data_Only( unsigned short data) 
{
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3)); //MSBFIRST SPI_MODE0
  LCD_CS_LO;
  vspi->transfer((data >> 0x08));  
  vspi->transfer((data & 0xFF));  
  LCD_CS_HI;
  vspi->endTransaction();
}


/*******************************************************************************
* Function Name  : LCD_ReadData
* Description    : LCD read data
* Input          : None
* Output         : None
* Return         : return data
* Attention      : None
*******************************************************************************/
unsigned short LCD_ReadData(void)
{ 
  unsigned short value;
  
  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, LSBFIRST, SPI_MODE3)); //MSBFIRST SPI_MODE0
  LCD_CS_LO;
  vspi->transfer(SPI_START | SPI_RD | SPI_DATA);    // Read: RS = 1, RW = 1         
  vspi->transfer(0);  

  value = vspi->transfer(0);                      // Read D8..D15                 
  value <<= 8;
  value |= vspi->transfer(0);                      // Read D0..D7                  

  LCD_CS_HI; 
  vspi->endTransaction();

  return value;
}


/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Writes to the selected LCD register.
* Input          : - index: address of the selected register.
*                  - data: value to write to the selected register.
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_WriteReg(unsigned short index, unsigned short data) {
  LCD_WriteIndex(index);
  LCD_WriteData(data);
}


/*******************************************************************************
* Function Name  : LCD_ReadReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
* Attention      : None
*******************************************************************************/
unsigned short LCD_ReadReg( unsigned short LCD_Reg)
{
  unsigned short LCD_RAM;
  
  /* Write 16-bit Index (then Read Reg) */
  LCD_WriteIndex(LCD_Reg);
  /* Read 16-bit Reg */
  LCD_RAM = LCD_ReadData();
  return LCD_RAM;
}


/*******************************************************************************
* Function Name  : LCD_Init_9320
* Description    : Initialize TFT Controller.
* Input          : Orientation
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Init_9320(int ori) {
  LCD_CS_HI;                
  LCD_RESET_HI;

  // Reset controller
  LCD_RESET_LO;
  delay(2);
  LCD_RESET_HI;
  delay(2);

  orien = ori;

  LCD_CS_LO; // Enable LCD

  LCD_WriteReg(0x00, 0x0000);
  LCD_WriteReg(0x01, 0x0100); /* Driver Output Contral */
  LCD_WriteReg(0x02, 0x0700); /* LCD Driver Waveform Contral */

  if (orien == VERTICAL) {
    LCD_WriteReg(0x0003, 0x1030); //Entry mode
    MAX_X = 240;
    MAX_Y = 320;
  } else {
    LCD_WriteReg(0x0003, 0x1008); //Entry mode 
    MAX_X = 320;
    MAX_Y = 240;
  }

  LCD_WriteReg(0x04, 0x0000); /* Scalling Contral */
  LCD_WriteReg(0x08, 0x0202); /* Display Contral 2 */
  LCD_WriteReg(0x09, 0x0000); /* Display Contral 3 */
  LCD_WriteReg(0x0a, 0x0000); /* Frame Cycle Contal */
  LCD_WriteReg(0x0c, (1 << 0)); /* Extern Display Interface Contral 1 */
  LCD_WriteReg(0x0d, 0x0000); /* Frame Maker Position */
  LCD_WriteReg(0x0f, 0x0000); /* Extern Display Interface Contral 2 */
  delay(50);
  LCD_WriteReg(0x07, 0x0101); /* Display Contral */
  delay(50);
  LCD_WriteReg(0x10, (1 << 12) | (0 << 8) | (1 << 7) | (1 << 6) | (0 << 4)); /* Power Control 1 */
  LCD_WriteReg(0x11, 0x0007);                             /* Power Control 2 */
  LCD_WriteReg(0x12, (1 << 8) | (1 << 4) | (0 << 0));     /* Power Control 3 */
  LCD_WriteReg(0x13, 0x0b00);                             /* Power Control 4 */
  LCD_WriteReg(0x29, 0x0000);                             /* Power Control 7 */
  LCD_WriteReg(0x2b, (1 << 14) | (1 << 4));

  LCD_WriteReg(0x50, 0);      /* Set X Start */
  LCD_WriteReg(0x51, 239);    /* Set X End */
  LCD_WriteReg(0x52, 0);      /* Set Y Start */
  LCD_WriteReg(0x53, 319);    /* Set Y End */

  delay(50);

  LCD_WriteReg(0x60, 0x2700); /* Driver Output Control */
  LCD_WriteReg(0x61, 0x0001); /* Driver Output Control */
  LCD_WriteReg(0x6a, 0x0000); /* Vertical Scroll Control */

  LCD_WriteReg(0x80, 0x0000); /* Display Position? Partial Display 1 */
  LCD_WriteReg(0x81, 0x0000); /* RAM Address Start? Partial Display 1 */
  LCD_WriteReg(0x82, 0x0000); /* RAM Address End-Partial Display 1 */
  LCD_WriteReg(0x83, 0x0000); /* Display Position? Partial Display 2 */
  LCD_WriteReg(0x84, 0x0000); /* RAM Address Start? Partial Display 2 */
  LCD_WriteReg(0x85, 0x0000); /* RAM Address End? Partial Display 2 */

  LCD_WriteReg(0x90, (0 << 7) | (16 << 0)); /* Frame Cycle Contral */
  LCD_WriteReg(0x92, 0x0000);        /* Panel Interface Contral 2 */
  LCD_WriteReg(0x93, 0x0001);        /* Panel Interface Contral 3 */
  LCD_WriteReg(0x95, 0x0110);        /* Frame Cycle Contral */
  LCD_WriteReg(0x97, (0 << 8));
  LCD_WriteReg(0x98, 0x0000);        /* Frame Cycle Contral */
  LCD_WriteReg(0x07, 0x0133);
  delay(50); 

  LCD_CS_HI;
     
  delay(30);
  LCD_CS_LO; 
}


/*******************************************************************************
* Function Name  : LCD_Init_9325
* Description    : Initialize TFT Controller.
* Input          : Orientation
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Init_9325(int ori) {
  LCD_CS_HI;                
  LCD_RESET_HI;

  // Reset controller
  LCD_RESET_LO;
  delay(2);
  LCD_RESET_HI;
  delay(2);

  orien = ori;

//  LANSCAPE
//  0x01 => 0x0100
//  0x03 => 0x1030
//  0x60 => 0x2700
//
//  PORTRAIT
//  0x01 => 0x0100
//  0x03 => 0x1038
//  0x60 => 0xA700
//
//  LANSCAPE_REV
//  0x01 => 0x0000
//  0x03 => 0x1030
//  0x60 => 0xA700
//
//  PORTRAIT_REV
//  0x01 => 0x0000
//  0x03 => 0x1038
//  0x60 => 0x2700

  LCD_CS_LO; // Enable LCD
  LCD_WriteReg(0xE5, 0x78F0); /* set SRAM internal timing */
  LCD_WriteReg(0x02, 0x0700); /* set 1 line inversion OLD 0700*/
  if (orien == VERTICAL) {
    LCD_WriteReg(0x01, 0x0000); /* set Driver Output Control OLD 0100*/
    LCD_WriteReg(0x03, 0x1038); //Entry mode OLD 1030
    LCD_WriteReg(0x60, 0x2700); /* Gate Scan Line */
    MAX_X = 240;
    MAX_Y = 320;
  } else {
    LCD_WriteReg(0x01, 0x0000); /* set Driver Output Control OLD 0100*/
    LCD_WriteReg(0x03, 0x1030); //Entry mode 
    LCD_WriteReg(0x60, 0x2700); /* Gate Scan Line */
    MAX_X = 320;
    MAX_Y = 240;
  }
  LCD_WriteReg(0x04, 0x0000); /* Resize register */
  LCD_WriteReg(0x08, 0x0207); /* set the back porch and front porch */
  LCD_WriteReg(0x09, 0x0000); /* set non-display area refresh cycle ISC[3:0] */
  LCD_WriteReg(0x0A, 0x0000); /* FMARK function */
  LCD_WriteReg(0x0C, 0x0000); /* RGB interface setting */
  LCD_WriteReg(0x0D, 0x0000); /* Frame marker Position */
  LCD_WriteReg(0x0F, 0x0000); /* RGB interface polarity */
  /*************Power On sequence ****************/
  LCD_WriteReg(0x10, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(0x11, 0x0007); /* DC1[2:0], DC0[2:0], VC[2:0] */
  LCD_WriteReg(0x12, 0x0000); /* VREG1OUT voltage */
  LCD_WriteReg(0x13, 0x0000); /* VDV[4:0] for VCOM amplitude */
  LCD_WriteReg(0x07, 0x0001); 
  delay(200); 
  /* Dis-charge capacitor power voltage */
  LCD_WriteReg(0x10, 0x1090); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(0x11, 0x0227); /* Set DC1[2:0], DC0[2:0], VC[2:0] */
  delay(50);               /* Delay 50ms */
  LCD_WriteReg(0x12, 0x001F); 
  delay(50);               /* Delay 50ms */
  LCD_WriteReg(0x13, 0x1500); /* VDV[4:0] for VCOM amplitude */
  LCD_WriteReg(0x29, 0x0027); /* 04 VCM[5:0] for VCOMH */
  LCD_WriteReg(0x2B, 0x000D); /* Set Frame Rate */
  delay(50);               /* Delay 50ms */
  LCD_WriteReg(0x20, 0x0000); /* GRAM horizontal Address */
  LCD_WriteReg(0x21, 0x0000); /* GRAM Vertical Address */
  /* ----------- Adjust the Gamma Curve ---------- */
  LCD_WriteReg(0x30, 0x0000);
  LCD_WriteReg(0x31, 0x0707);
  LCD_WriteReg(0x32, 0x0307);
  LCD_WriteReg(0x35, 0x0200);
  LCD_WriteReg(0x36, 0x0008);
  LCD_WriteReg(0x37, 0x0004);
  LCD_WriteReg(0x38, 0x0000);
  LCD_WriteReg(0x39, 0x0707);
  LCD_WriteReg(0x3C, 0x0002);
  LCD_WriteReg(0x3D, 0x1D04);
  /* ------------------ Set GRAM area --------------- */
  LCD_WriteReg(0x50, 0);      /* Set X Start Horizontal GRAM Start Address */
  LCD_WriteReg(0x51, 239);    /* Set X End Horizontal GRAM End Address */
  LCD_WriteReg(0x52, 0);      /* Set Y Start Vertical GRAM Start Address */ 
  LCD_WriteReg(0x53, 319);    /* Set Y End Vertical GRAM Start Address */
  
  LCD_WriteReg(0x61, 0x0001); /* NDL, VLE, REV */
  LCD_WriteReg(0x6A, 0x0000); /* set scrolling line */
  /* -------------- Partial Display Control --------- */
  LCD_WriteReg(0x80, 0x0000);
  LCD_WriteReg(0x81, 0x0000);
  LCD_WriteReg(0x82, 0x0000);
  LCD_WriteReg(0x83, 0x0000);
  LCD_WriteReg(0x84, 0x0000);
  LCD_WriteReg(0x85, 0x0000);
  /* -------------- Panel Control ------------------- */
  LCD_WriteReg(0x90, 0x0010);
  LCD_WriteReg(0x92, 0x0600);
  LCD_WriteReg(0x07, 0x0133); /* 262K color and display ON */
}

  
/*******************************************************************************
* Function Name  : LCD_SetCursor
* Description    : Sets the cursor position.
* Input          : - x: specifies the X position.
*                  - y: specifies the Y position. 
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_SetCursor(unsigned int x, unsigned int y) {
  if (orien == VERTICAL) {
    LCD_WriteReg(0x0020, x);
    LCD_WriteReg(0x0021, y);
  } else {
    LCD_WriteReg(0x0020, 240 - y);
    LCD_WriteReg(0x0021, x);
  }
}


/******************************************************************************
* Function Name  : LCD_BGR2RGB
* Description    : RRRRRGGGGGGBBBBB To BBBBBGGGGGGRRRRR
* Input          : - color: BRG Color value  
* Output         : None
* Return         : RGB Color value
* Attention      : None
*******************************************************************************/
static unsigned short LCD_BGR2RGB( unsigned short color)
{
  unsigned short  r, g, b, rgb;
  
  b = ( color>>0 )  & 0x1f;
  g = ( color>>5 )  & 0x3f;
  r = ( color>>11 ) & 0x1f;
  
  rgb =  (b<<11) + (g<<5) + (r<<0);
  
  return( rgb );
}


/******************************************************************************
* Function Name  : LCD_GetPoint
* Description    : Get color value for the specified coordinates
* Input          : - Xpos: Row Coordinate
*                  - Xpos: Line Coordinate 
* Output         : None
* Return         : Screen Color
* Attention      : None
*******************************************************************************/
unsigned short LCD_GetPoint( unsigned short Xpos, unsigned short Ypos)
{
  unsigned short dummy;
  
  LCD_SetCursor(Xpos,Ypos);

  LCD_WriteIndex(0x0022);  
  
  dummy = LCD_ReadData();   /* An empty read */
  dummy = LCD_ReadData(); 

  return  LCD_BGR2RGB( dummy );
}


/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : Drawn at a specified point coordinates
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_SetPoint( unsigned short Xpos, unsigned short Ypos, unsigned short point)
{
  if( Xpos >= MAX_X || Ypos >= MAX_Y )
  {
    return;
  }
  LCD_SetCursor(Xpos,Ypos);
  LCD_WriteReg(0x0022,point);
}


/******************************************************************************
* Function Name  : LCD_ResetWindow
* Description    : Reset window to max value
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_ResetWindow() {
  LCD_WriteReg(0x0050, 0x0000); //Horizontal Address Start Position  
  LCD_WriteReg(0x0051, 0x00EF); //Horizontal Address end Position (239)  
  LCD_WriteReg(0x0052, 0x0000); //Vertical Address Start Position  
  LCD_WriteReg(0x0053, 0x013F); //Vertical Address end Position (319) 
}


/******************************************************************************
* Function Name  : LCD_SetWindow
* Description    : Set window to value
* Input          : starting, ending coord  
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_SetWindow(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  LCD_WriteReg(0x0050, x1); //Horizontal Address Start Position  
  LCD_WriteReg(0x0051, x2); //Horizontal Address end Position  
  LCD_WriteReg(0x0052, y1); //Vertical Address Start Position  
  LCD_WriteReg(0x0053, y2); //Vertical Address end Position                     
}


/******************************************************************************
* Function Name  : LCD_HorLine
* Description    : Draw horizontal line
* Input          : starting coord ending y
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_HorLine(unsigned short x1, unsigned short x2, unsigned short y, unsigned int color) {
  LCD_DrawLine(x1,y,x2,y,color);
}


/******************************************************************************
* Function Name  : LCD_VerLine
* Description    : Draw horizontal line
* Input          : starting coord ending x
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_VerLine(unsigned short y1, unsigned short y2, unsigned short x, unsigned int color){
  LCD_DrawLine(x,y1,x,y2,color);  
}  


/******************************************************************************
* Function Name  : LCD_FillRectangle
* Description    : Draw filled rectangle
* Input          : starting, ending coord border color
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_FillRectangle(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2, int color){      
  LCD_DrawBox(x1, y1, x2, y2, color, color);
}


/******************************************************************************
* Function Name  : LCD_Rectangle
* Description    : Draw empty rectangle
* Input          : starting, ending coord border color
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Rectangle(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2, int color){      
  LCD_VerLine ( y1, y2, x1, color ); 
  LCD_VerLine ( y1, y2, x2, color ); 
  LCD_HorLine ( x1, x2, y1, color ); 
  LCD_HorLine ( x1, x2, y2, color ); 
}


/******************************************************************************
* Function Name  : LCD_Circle
* Description    : Draw empty circle
* Input          : center coord, radius, border color
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_Circle(int cx, int cy, int radius, uint16_t color){
  int error = -radius;
  int x = radius;
  int y = 0;
 
  while (x >= y){
    plot8points(cx, cy, x, y, color);
 
    error += y;
    ++y;
    error += y;

    if (error >= 0){
      error -= x;
      --x;
      error -= x;
    }
  }
}


void plot4points(int cx, int cy, int x, int y, uint16_t color) {
  LCD_SetPoint(cx + x, cy + y, color);

  if (x != 0) {
    LCD_SetPoint(cx - x, cy + y, color);
  }

  if (y != 0) {
    LCD_SetPoint(cx + x, cy - y, color);
  }

  if (x != 0 && y != 0) {
    LCD_SetPoint(cx - x, cy - y, color);
  }
}


void plot8points(int cx, int cy, int x, int y, uint16_t color){
  plot4points(cx, cy, x, y, color);
  if (x != y) {
    plot4points(cx, cy, y, x, color);
  }
}


/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : Fill the screen as the specified color
* Input          : - Color: Screen Color
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void LCD_Clear(unsigned int color) {
  if (orien == HORIZONTAL) LCD_SetCursor(319, 0);
  else LCD_SetCursor(0, 0);
  LCD_WriteIndex(0x0022);
  LCD_CS_LO; 
  LCD_Write_Data_Start();
  for (long i = 0; i <= MAX_X * MAX_Y; i++) {
    LCD_WriteData(color);
  }
  LCD_CS_HI;
}


/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : Bresenham's line algorithm
* Input          : - x1: A point line coordinates
*                  - y1: A point column coordinates 
*      - x2: B point line coordinates
*      - y2: B point column coordinates 
*      - color: Line color
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/   
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t col)
{
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  int err = (dx>dy ? dx : -dy)/2, e2;
 
  for(;;){
    LCD_SetPoint(x0, y0, col);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}


/******************************************************************************
* Function Name  : LCD_DrawBox
* Description    : Multiple line  makes box
* Input          : - x1: A point line coordinates upper left corner
*                  - y1: A point column coordinates
*                  - x2: B point line coordinates lower right corner
*                  - y2: B point column coordinates
*                  - col: Line color
*                  - fcol: fill color -1 no color
* Output         : None
* Return         : None
******************************************************************************/
void LCD_DrawBox(unsigned short x0, unsigned short y0, unsigned short x1, 
unsigned short y1 , unsigned short col, int fcol)
{
  unsigned short i, xx0, xx1, yy0;

  LCD_DrawLine(x0, y0, x1, y0, col);
  LCD_DrawLine(x1, y0, x1, y1, col);
  LCD_DrawLine(x0, y0, x0, y1, col);
  LCD_DrawLine(x0, y1, x1, y1, col);

  if  (fcol != -1)
  {
    for (i = 0; i < y1 - y0 - 1; i++)
    {
      xx0 = x0 + 1;
      yy0 = y0 + 1 + i;
      xx1 = x1 - 1;
      //LCD_DrawLine(xx0, yy0, xx1, yy0, (unsigned short)fcol);
      for (int i = xx0; i <= xx1; i++) LCD_SetPoint(i, yy0, fcol);
    }
  }
}


/******************************************************************************
* Function Name  : PutChar
* Description    : Lcd screen displays a character
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - ASCI: Displayed character
*          - charColor: Character color  
*          - bkColor: Background color 
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void PutChar( uint16_t Xpos, uint16_t Ypos, int ASCI, uint16_t charColor, uint16_t bkColor )
{

  uint16_t i, j;
  unsigned int offset = ASCI;
  uint8_t tmp_char;
  for ( i = 0; i < 12; i++ )
  {
    tmp_char = pgm_read_word(&Smalls[offset + i]);
    for ( j = 0; j < 8; j++ )
    {
      if ( (tmp_char >> 7 - j) & 0x01 == 0x01 )
      {
        LCD_SetPoint( Xpos + j, Ypos + i, charColor );
      }
      else
      {
        LCD_SetPoint( Xpos + j, Ypos + i, bkColor );
      }
    }
  }
}


/******************************************************************************
* Function Name  : PutChar32
* Description    : Lcd screen displays a character
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - ASCI: Displayed character
*          - charColor: Character color  
*          - bkColor: Background color 
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void PutChar32( uint16_t Xpos, uint16_t Ypos, int ASCI, uint16_t charColor, uint16_t bkColor )
{
  uint16_t i, j;
  uint8_t tmp_char;
  unsigned int offset = ASCI;
  for ( i = 0; i < 32; i++ )
  {
    tmp_char = pgm_read_word(&chlib[offset + i]);
    for ( j = 0; j < 8; j++ )
    {
      if ( (tmp_char >> 7 - j) & 0x01 == 0x01 )
      {
        LCD_SetPoint( Xpos + j, Ypos + i, charColor );  /* Character color */
      }
      else
      {
        LCD_SetPoint( Xpos + j, Ypos + i, bkColor );  /* Background color */
      }
    }
    i++;
    tmp_char = pgm_read_word(&chlib[offset + i]);
    for ( j = 0; j < 8; j++ )
    {
      if ( (tmp_char >> 7 - j) & 0x01 == 0x01 )
      {
        LCD_SetPoint( Xpos + j + 8, Ypos + i - 1, charColor );  /* Character color */
      }
      else
      {
        LCD_SetPoint( Xpos + j + 8, Ypos + i - 1, bkColor ); /* Background color */
      }
    }
  }
}


/******************************************************************************
* Function Name  : NumChar
* Description    : Lcd screen displays a character
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - ASCI: Displayed character
*          - charColor: Character color  
*          - bkColor: Background color 
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void NumChar( uint16_t Xpos, uint16_t Ypos, int ASCI, uint16_t charColor, uint16_t bkColor )
{
  uint16_t i, j, w;

  uint8_t tmp_char;

  unsigned int offset = ASCI;
  for ( i = 0; i < 50; i++ )
  {
    for ( w = 0; w < 4; w++ )
    {
      tmp_char = pgm_read_word(&SevenSegNumFont[offset + (i * 4) + w]);
      for ( j = 0; j < 8; j++ )
      {
        if ( (tmp_char >> 7 - j) & 0x01 == 0x01 )
        {
          LCD_SetPoint( Xpos + j + (w * 8), Ypos + i, charColor ); /* Character color */
        }
        else
        {
          LCD_SetPoint( Xpos + j + (w * 8), Ypos + i, bkColor ); /* Background color */
        }
      }
    }
  }
}


/******************************************************************************
* Function Name  : NumChar24
* Description    : Lcd screen displays a character
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - ASCI: Displayed character
*          - charColor: Character color  
*          - bkColor: Background color 
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void NumChar24( uint16_t Xpos, uint16_t Ypos, int ASCI, uint16_t charColor, uint16_t bkColor )
{
    uint16_t i, j, w;
 
    uint8_t tmp_char;
 
    unsigned int offset = ASCI;
    for( i=0; i<24; i++ )
    {
        //for( w=0; w<4; w++ )
        for( w=0; w<6; w++ )
        {
           tmp_char = pgm_read_word(&BatteryFont[offset+(i*6)+w]);
           for( j=0; j<8; j++ )
           {
               if( (tmp_char >> 7 - j) & 0x01 == 0x01 )
               {
                   LCD_SetPoint( Xpos + j + (w*8), Ypos + i, charColor );  /* Character color */
               }
               else
               {
                   LCD_SetPoint( Xpos + j + (w*8), Ypos + i, bkColor );  /* Background color */
               }
           }
        }
    }
}


/******************************************************************************
* Function Name  : LCD_Text
* Description    : Displays the string big font 
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - str: Displayed string
*          - charColor: Character color   
*          - bkColor: Background color  
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void LCD_Text(uint16_t Xpos, uint16_t Ypos, const char *str,uint16_t Color, uint16_t bkColor)
{
    int l = 0;
    uint16_t v;
    unsigned int Chnum=0;
    do
    {
        v = *(str+l);
        l++;  
        Chnum = (v-32)*32;
        PutChar32( Xpos, Ypos, Chnum, Color, bkColor );
        if( Xpos < MAX_X - 32 )
        {
            Xpos += 16;
        } 
        else if ( Ypos < MAX_Y - 16 )
        {
            Xpos = 10;
            Ypos += 32;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }    
    }
    while ( l < strlen(str));
}


/******************************************************************************
* Function Name  : LCD_Num
* Description    : Displays 7 segment number
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - str: Displayed string
*          - charColor: Character color   
*          - bkColor: Background color  
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void LCD_Num(uint16_t Xpos, uint16_t Ypos, const char *str, uint16_t Color, uint16_t bkColor)
{
    int l = 0;
    uint16_t v;
    unsigned int Chnum=0;
    do
    {
        v = *(str+l);
        l++;  
        Chnum = (v-32)*200;
        if (Chnum == 65136) {
            Chnum=448;
            PutChar32( Xpos, Ypos+20, Chnum, Color, bkColor );
            Xpos += 16;
        }
        else {
            NumChar( Xpos, Ypos, Chnum, Color, bkColor );
            if( Xpos < MAX_X - 32 )
            {
                  Xpos += 32;
            } 
            else if ( Ypos < MAX_Y - 50 )
            {
                Xpos = 10;
                Ypos += 50;
            }   
            else
            {
                Xpos = 0;
                Ypos = 0;
            }
        }    
    }
    while ( l < strlen(str));
}


/******************************************************************************
* Function Name  : LCD_Battery
* Description    : Displays Battery status using font
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - str: Displayed string
*          - charColor: Character color   
*          - bkColor: Background color  
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void LCD_Battery(uint16_t Xpos, uint16_t Ypos, const char *str, uint16_t Color, uint16_t bkColor)
{
    int l = 0;
    uint16_t v;
    unsigned int Chnum=0;
    do
    {
        v = *(str+l);
        l++;  
        Chnum = (v-48)*144;
        NumChar24( Xpos, Ypos, Chnum, Color, bkColor );
        if( Xpos < MAX_X - 48 )
        {
              Xpos += 48;
        } 
        else if ( Ypos < MAX_Y - 24 )
        {
            Xpos = 10;
            Ypos += 24;
        }   
        else
        {
            Xpos = 0;
            Ypos = 0;
        }
    }
    while ( l < strlen(str));
}


/******************************************************************************
* Function Name  : LCD_Small
* Description    : Displays the string smoll font
* Input          : - Xpos: Horizontal coordinate
*                  - Ypos: Vertical coordinate 
*          - str: Displayed string
*          - charColor: Character color   
*          - bkColor: Background color  
* Output         : None
* Return         : None
* Attention    : None
*******************************************************************************/
void LCD_Small(uint16_t Xpos, uint16_t Ypos, const char *str, uint16_t Color, uint16_t bkColor)
{
  int l = 0;
  uint16_t v;
  unsigned int Chnum = 0;

  do
  {
    v = *(str + l);
    l++;
    Chnum = (v - 32) * 12;
    PutChar( Xpos, Ypos, Chnum, Color, bkColor );
    if ( Xpos < MAX_X - 16 )
    {
      Xpos += 8;
    }
    else if ( Ypos < MAX_Y - 12 )
    {
      Xpos = 10;
      Ypos += 24;
    }
    else
    {
      Xpos = 0;
      Ypos = 0;
    }
  }
  while ( l < strlen(str));
}


/*******************************************************************************
* Function Name  : LCD_DisplayOn
* Description    : Switch display on via software
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_DisplayOn(void)
{
  LCD_WriteReg(0x07, 0x0173);
}


/*******************************************************************************
* Function Name  : LCD_DisplayOff
* Description    : Switch display off via software
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_DisplayOff(void)
{
  LCD_WriteReg(0x07, 0x0000);
}


/*******************************************************************************
* Function Name  : DrawCross
* Description    : specified coordinates painting crosshairs
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void DrawCross(unsigned int Xpos, unsigned int Ypos, unsigned int color)
{
  int lar = 3;

  LCD_DrawLine(Xpos - lar, Ypos, Xpos - 4, Ypos, color);
  LCD_DrawLine(Xpos + 4, Ypos, Xpos + lar, Ypos, color);
  LCD_DrawLine(Xpos, Ypos - lar, Xpos, Ypos - 4, color);
  LCD_DrawLine(Xpos, Ypos + 4, Xpos, Ypos + lar, color);
}


/*******************************************************************************
* Function Name  : UTouch::UTouch
* Description    : set CS & IRQ for touch
* Input          : - tcs: CS
*                  - irq: IRQ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
UTouch::UTouch(byte tcs, byte irq)
{
  T_CS = tcs;
  T_IRQ = irq;
}


/*******************************************************************************
* Function Name  : UTouch::InitTouch
* Description    : initialization of the touch screen
* Input          : - orientation: orientation of the LCD
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void UTouch::InitTouch(byte orientation)
{
  orient = orientation;

  if (orient == VERTICAL) {
    disp_x_size = 239;    // do not forget them if different
    disp_y_size = 319;    // do not forget them if different
  } else {
    disp_x_size = 319;    // do not forget them if different
    disp_y_size = 239;    // do not forget them if different
  }

  prec = 10;

  //reset IRQ
  touch_ReadData(0x91);
  //set IRQ
  touch_ReadData(0x90);

  /*Serial.println("SUMMARY");
  Serial.print("disp_x_size=");
  Serial.println(disp_x_size);
  Serial.print("disp_y_size=");
  Serial.println(disp_y_size);
  Serial.println("");*/
}


/*******************************************************************************
* Function Name  : UTouch::touch_ReadData
* Description    : reading the touch screen
* Input          : - command: command to send
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
word UTouch::touch_ReadData(byte command)
{
  //first send the command, this will not return anything useful
  //so discard the return byte
 
  vspi2->beginTransaction(SPISettings(spiClk2, MSBFIRST, SPI_MODE0));
  TP_CS_LO;
  vspi2->transfer(command);  
  //delay(1);

  //next we need to send 12 bits of nothing to get the 12 bits we need back
  //we can only send in batches of 8, ie 1 byte so do this twice
  byte f1 = vspi2->transfer(0x00);
  //delay(1);
  byte f2 = vspi2->transfer(0x00);

  TP_CS_HI; 
  vspi2->endTransaction();
   
  //combine the 16 bits into a word
  word w = f1;
  w <<= 8;
  w += f2;
  w >>= 4;
  w &= 0x0fff;

  //word w = word(f1, f2);
  //the word is organised with MSB leftmost, shift right by 3 to make 12 bits
  //w = w >> 4;

  //and return the result
  return w;
}


/*******************************************************************************
* Function Name  : UTouch::read
* Description    : X Y obtained after filtering
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void UTouch::read()
{
  Coord screen;
  int m0, m1, m2, temp[3];
  unsigned char count = 0;
  int buffer[2][9] = {{0}, {0}}; /* Multiple sampling coordinates X and Y */

  /* Loop sampling 9 times */
  do {
    if (myTouch.dataAvailable()) {
      myTouch.read_HW();
      buffer[0][count] = TP_X;
      buffer[1][count] = TP_Y;
      count++;
    }
  }
  while ( !myTouch.dataAvailable() && count < 9 );

  if ( count == 9 )   /* Successful sampling 9, filtering */
  {
    /* In order to reduce the amount of computation, were divided into three groups averaged */
    temp[0] = ( buffer[0][0] + buffer[0][1] + buffer[0][2] ) / 3;
    temp[1] = ( buffer[0][3] + buffer[0][4] + buffer[0][5] ) / 3;
    temp[2] = ( buffer[0][6] + buffer[0][7] + buffer[0][8] ) / 3;
    /* Calculate the three groups of data */
    m0 = temp[0] - temp[1];
    m1 = temp[1] - temp[2];
    m2 = temp[2] - temp[0];
    /* Absolute value of the above difference */
    m0 = m0 > 0 ? m0 : (-m0);
    m1 = m1 > 0 ? m1 : (-m1);
    m2 = m2 > 0 ? m2 : (-m2);
    /* Judge whether the absolute difference exceeds the difference between the threshold,
       If these three absolute difference exceeds the threshold,
         The sampling point is judged as outliers, Discard sampling points */
    if ( m0 > THRESHOLD  &&  m1 > THRESHOLD  &&  m2 > THRESHOLD ) {
      return;
    }
    /* Calculating their average value */
    if ( m0 < m1 ) {
      if ( m2 < m0 ) {
        screen.x = ( temp[0] + temp[2] ) / 2;
      } else {
        screen.x = ( temp[0] + temp[1] ) / 2;
      }
    } else if (m2 < m1) {
      screen.x = ( temp[0] + temp[2] ) / 2;
    } else {
      screen.x = ( temp[1] + temp[2] ) / 2;
    }

    /* calculate the average value of Y */
    temp[0] = ( buffer[1][0] + buffer[1][1] + buffer[1][2] ) / 3;
    temp[1] = ( buffer[1][3] + buffer[1][4] + buffer[1][5] ) / 3;
    temp[2] = ( buffer[1][6] + buffer[1][7] + buffer[1][8] ) / 3;

    m0 = temp[0] - temp[1];
    m1 = temp[1] - temp[2];
    m2 = temp[2] - temp[0];

    m0 = m0 > 0 ? m0 : (-m0);
    m1 = m1 > 0 ? m1 : (-m1);
    m2 = m2 > 0 ? m2 : (-m2);
    if ( m0 > THRESHOLD && m1 > THRESHOLD && m2 > THRESHOLD ) {
      return;
    }

    if ( m0 < m1 ) {
      if ( m2 < m0 ) {
        screen.y = ( temp[0] + temp[2] ) / 2;
      } else {
        screen.y = ( temp[0] + temp[1] ) / 2;
      }
    } else if ( m2 < m1 ) {
      screen.y = ( temp[0] + temp[2] ) / 2;
    } else {
      screen.y = ( temp[1] + temp[2] ) / 2;
    }

    Screen.x = screen.x;
    Screen.y = screen.y;
    return;
  }
  return;
}


/*******************************************************************************
* Function Name  : UTouch::read_HW
* Description    : hardware reading
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void UTouch::read_HW()
{
  unsigned long tx = 0, temp_x = 0;
  unsigned long ty = 0, temp_y = 0;
  int datacount = 0;
  uint32_t now = millis();

 
  for (int i = 0; i < prec; i++) {
    temp_x = touch_ReadData(0x90); //90
    temp_y = touch_ReadData(0xD0); //D0
    if (!((temp_x == 0) or (temp_y == 0))) {
      ty += temp_x;
      tx += temp_y;
      datacount++;
    }
  }

  if (datacount > 0) {
    if (orien == VERTICAL) {
      Screen.x = TP_X = tx / datacount;
      Screen.y = TP_Y = ty / datacount;
    } else {
      Screen.x = TP_X = ty / datacount;
      Screen.y = TP_Y = tx / datacount;
    }
  } else {
    TP_X = -1;
    TP_Y = -1;
  }
}


/*******************************************************************************
* Function Name  : UTouch::dataAvailable
* Description    : check if TS is touched
* Input          : None
* Output         : None
* Return         : true if touched false if not
* Attention      : None
*******************************************************************************/
bool UTouch::dataAvailable()
{
  bool avail = 0;

  avail = !digitalRead(TP_IRQ);
  if (avail) digitalWrite(ONE_LED, HIGH);
  else digitalWrite(ONE_LED, LOW);
  return avail;
}

/*******************************************************************************
* Function Name  : UTouch::setPrecision
* Description    : set precision of the touch
* Input          : - precision: precision of touch
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void UTouch::setPrecision(byte precision)
{
  switch (precision)
  {
    case PREC_LOW:
      prec = 1;
      break;
    case PREC_MEDIUM:
      prec = 10;
      break;
    case PREC_HI:
      prec = 25;
      break;
    case PREC_EXTREME:
      prec = 100;
      break;
    default:
      prec = 10;
      break;
  }
}


/*******************************************************************************
* Function Name  : setCalibrationMatrix
* Description    : Calculated K A B C D E F
* Input          : None
* Output         : None
* Return         : return 1 success , return 0 fail
* Attention      : None
*******************************************************************************/
int setCalibrationMatrix( Coord * displayPtr, Coord * screenPtr, Matrix3 * matrixPtr)
{

  int retTHRESHOLD = ENABLE ;

  matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                       ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
  if ( matrixPtr->Divider == 0 )
  {
    retTHRESHOLD = DISABLE;
  }
  else
  {

    matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                    ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

    matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) -
                    ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

    matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

    matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) -
                    ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;

    matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) -
                    ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

    matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                    (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                    (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
  }
  return ( retTHRESHOLD ) ;
}


/*******************************************************************************
* Function Name  : getDisplayPoint
* Description    : channel XY via K A B C D E F value converted to the LCD screen coordinates
* Input          : None
* Output         : None
* Return         : return 1 success , return 0 fail
* Attention      : None
*******************************************************************************/
int getDisplayPoint()
{
  int retTHRESHOLD = ENABLE ;
  long double an, bn, cn, dn, en, fn, sx, sy, md;
  int i;

  an = matrix.An;
  bn = matrix.Bn;
  cn = matrix.Cn;
  dn = matrix.Dn;
  en = matrix.En;
  fn = matrix.Fn;

  sx = Screen.x;
  sy = Screen.y;
  md = matrix.Divider;

  if ( md != 0 ) {
    /* XD = AX+BY+C */
    disp.x = ( (an * sx) + (bn * sy) + cn) / md;
    /* YD = DX+EY+F */
    disp.y = ( (dn * sx) + (en * sy) + fn) / md;
    for (i = 0; i < MAXBUTTON; i++) {
      if (Butt[i].exist) {
        if ((disp.x>Butt[i].x0)&&(disp.x<Butt[i].x1)&&(disp.y>Butt[i].y0)&&(disp.y<Butt[i].y1)) Butt[i].pressed=1;
        else Butt[i].pressed=0;
      }
    }
  } else {
    retTHRESHOLD = DISABLE;
  }
  return (retTHRESHOLD);
}


/*******************************************************************************
* Function Name  : calibrateRead
* Description    : read calibrated data for touch screen from SPIFFS
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void UTouch::calibrateRead()
{
  unsigned char i;
  Coord * Ptr;
  byte b2[sizeof(Matrix3)]; // create byte array to store the struct
  uint8_t testV, testH;
  bool avail;
  
  if (orien == VERTICAL) {
    DisplaySample[1].y = 310;
    DisplaySample[2].y = 190;

    testV = existFile(SPIFFS, "/vertical.cal");

    // read the values
    if ( !testV ) { //data matrix not exist -> proceed to calibration
      LCD_Clear(BLACK);
      //LCD_Small(15, 15, "Touch crosshair to calibrate", WHITE, BLACK);

      for (i = 0; i < 3; i++)
      {
        DrawCross(DisplaySample[i].x, DisplaySample[i].y, WHITE);
        do {
          delay(100);
        } while (!myTouch.dataAvailable()); // wait for touch
        myTouch.read();
        ScreenSample[i].x = TP_X;
        ScreenSample[i].y = TP_Y;
        delay(1000);
      }

      // set calibration parameters
      setCalibrationMatrix(&DisplaySample[0], &ScreenSample[0], &matrix);

      Screen.x = -1;
      Screen.y = -1;

      // write the values
      memcpy(b2, &matrix, sizeof(Matrix3)); // copy the struct to the byte array
      writeFileBin(SPIFFS, "/vertical.cal", (uint8_t*) b2, sizeof(Matrix3));
    } else {
      // read &matrix, sizeof(Matrix3)
      readFileBin(SPIFFS, "/vertical.cal", (uint8_t*) b2, sizeof(Matrix3));
      memcpy(&matrix, b2, sizeof(Matrix3)); // copy byte array to temporary struct
    }
  } else {
    DisplaySample[1].y = 230;
    DisplaySample[2].y = 190;

    testH = existFile(SPIFFS, "/horizontal.cal");

    // read the values
    if ( !testH ) { //data matrix not exist -> proceed to calibration
      LCD_Clear(BLACK);
      LCD_Small(15, 15, "Touch crosshair to calibrate", WHITE, BLACK);

      for (i = 0; i < 3; i++)
      {
        DrawCross(DisplaySample[i].x, DisplaySample[i].y, WHITE);
        do {
          delay(100);
        } while (!myTouch.dataAvailable()); // wait for touch
        myTouch.read();
        ScreenSample[i].x = TP_X;
        ScreenSample[i].y = TP_Y;
        delay(1000);
      }

      // set calibration parameters
      setCalibrationMatrix(&DisplaySample[0], &ScreenSample[0], &matrix);

      Screen.x = -1;
      Screen.y = -1;

      // write the values
      memcpy(b2, &matrix, sizeof(Matrix3)); // copy the struct to the byte array
      writeFileBin(SPIFFS, "/horizontal.cal", (uint8_t*) b2, sizeof(Matrix3));
    } else {
      // read &matrix, sizeof(Matrix3)
      readFileBin(SPIFFS, "/horizontal.cal", (uint8_t*) b2, sizeof(Matrix3));
      memcpy(&matrix, b2, sizeof(Matrix3)); // copy byte array to temporary struct
    }
  }
}


/*******************************************************************************
* Function Name  : TP_Button
* Description    : Return the button pressed
* Input          : None
* Output         : None
* Return         : Button pressed -1 if no button pressed
* Attention      : None
*******************************************************************************/
int TP_Button()
{
  int i;

  for (i = 0; i < MAXBUTTON; i++)
  {
    if (Butt[i].exist)
    {
      if (Butt[i].pressed == 1)
      {
        LCD_DrawBox(Butt[i].x0, Butt[i].y0, Butt[i].x1, Butt[i].y1, Butt[i].fcol, Butt[i].col);
        LCD_Small(Butt[i].x0 + Butt[i].xo, Butt[i].y0 + Butt[i].yo, Butt[i].text, Butt[i].fcol, Butt[i].col);
        delay(50);
        LCD_DrawBox(Butt[i].x0, Butt[i].y0, Butt[i].x1, Butt[i].y1 , Butt[i].col, Butt[i].fcol);
        LCD_Small(Butt[i].x0 + Butt[i].xo, Butt[i].y0 + Butt[i].yo, Butt[i].text, Butt[i].col, Butt[i].fcol);
        Butt[i].pressed = 0;
        return i;
      }
    }
  }
  return -1;
}


/*******************************************************************************
* Function Name  : TP_ResetButton
* Description    : Reset buttons
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void TP_ResetButton()
{
  int i;

  for (i = 0; i < MAXBUTTON; i++)
    Butt[i].exist = 0;
}


/******************************************************************************
* Function Name  : LCD_Button
* Description    : Make button
* Input          : - x0: A point line coordinates upper left corner
*                  - y0: A point column coordinates
*                  - x1: B point line coordinates lower right corner
*                  - y1: B point column coordinates
*                  - col: Line color
*                  - fcol: fill color -1 means no fill
*                  - xo: text x origin
*                  - yo: text y origin
*                  - text: the text
*                  - buttn: number of button
* Output         : None
* Return         : None
******************************************************************************/
void LCD_Button(unsigned short x0, unsigned short y0, unsigned short x1, 
unsigned short y1, unsigned short col, int fcol, unsigned short xo, 
unsigned short yo, char* text, unsigned short buttn)
{
  LCD_DrawBox(x0, y0, x1, y1 , col, fcol);
  LCD_Small(x0 + xo, y0 + yo, text, col, fcol);
  Butt[buttn].exist = 1;
  Butt[buttn].x0 = x0;
  Butt[buttn].y0 = y0;
  Butt[buttn].x1 = x1;
  Butt[buttn].y1 = y1;
  Butt[buttn].col = col;
  Butt[buttn].fcol = fcol;
  Butt[buttn].xo = xo;
  Butt[buttn].yo = yo;
  strcpy(Butt[buttn].text, text);
  Butt[buttn].pressed = 0;
}


/******************************************************************************
* Function Name  : LCD_DrawKeypad
* Description    : draw a keypad
* Input          : - xoff: x coordinate upper left corner
*                  - yoff: y coordinate upper left corner
* Output         : None
* Return         : None
******************************************************************************/
void LCD_DrawKeypad(int xoff, int yoff) {
  LCD_Button(5 + xoff, 5 + yoff, 35 + xoff, 35 + yoff, WHITE, BLUE, 5, 5, "7", 7);
  LCD_Button(40 + xoff, 5 + yoff, 70 + xoff, 35 + yoff, WHITE, BLUE, 5, 5, "8", 8);
  LCD_Button(75 + xoff, 5 + yoff, 105 + xoff, 35 + yoff, WHITE, BLUE, 5, 5, "9", 9);
  //LCD_Button(110+xoff,5+yoff,140+xoff,35+yoff,WHITE,BLUE,5,5,"Bck",12);
  LCD_Button(110 + xoff, 5 + yoff, 140 + xoff, 70 + yoff, WHITE, BLUE, 5, 5, "Bck", 12);

  LCD_Button(5 + xoff, 40 + yoff, 35 + xoff, 70 + yoff, WHITE, BLUE, 5, 5, "4", 4);
  LCD_Button(40 + xoff, 40 + yoff, 70 + xoff, 70 + yoff, WHITE, BLUE, 5, 5, "5", 5);
  LCD_Button(75 + xoff, 40 + yoff, 105 + xoff, 70 + yoff, WHITE, BLUE, 5, 5, "6", 6);
  //LCD_Button(110+xoff,40+yoff,140+xoff,70+yoff,WHITE,BLUE,5,5,"Clr",13);
  LCD_Button(110 + xoff, 75 + yoff, 140 + xoff, 140 + yoff, WHITE, BLUE, 5, 5, "Clr", 13);

  LCD_Button(5 + xoff, 75 + yoff, 35 + xoff, 105 + yoff, WHITE, BLUE, 5, 5, "1", 1);
  LCD_Button(40 + xoff, 75 + yoff, 70 + xoff, 105 + yoff, WHITE, BLUE, 5, 5, "2", 2);
  LCD_Button(75 + xoff, 75 + yoff, 105 + xoff, 105 + yoff, WHITE, BLUE, 5, 5, "3", 3);

  LCD_Button(5 + xoff, 110 + yoff, 35 + xoff, 140 + yoff, WHITE, BLUE, 5, 5, "000", 10);
  LCD_Button(40 + xoff, 110 + yoff, 70 + xoff, 140 + yoff, WHITE, BLUE, 5, 5, "00", 11);
  LCD_Button(75 + xoff, 110 + yoff, 105 + xoff, 140 + yoff, WHITE, BLUE, 5, 5, "0", 0);
}


/******************************************************************************
* Function Name  : listDir
* Description    : list the fs content of SPIFFS
* Input          : - fs: pointer to fs
*                  - dirname: directory name
*                  - level: directory levels
* Output         : None
* Return         : None
******************************************************************************/
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}


/******************************************************************************
* Function Name  : existFile
* Description    : check if file exist SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
* Output         : 1 if file exist, 0 if file doesn't exist
* Return         : None
******************************************************************************/
unsigned int existFile(fs::FS &fs, const char * path){
    Serial.printf("Checking file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- file does NOT exist");
        return(0);
    }

    Serial.println("- file exist");
    return(1); 
}


/******************************************************************************
* Function Name  : readFile
* Description    : read ascii file from SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
* Output         : none
* Return         : None
******************************************************************************/
void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
}


/******************************************************************************
* Function Name  : writeFile
* Description    : write ascii file to SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
*                  - message: ascii string to write 
* Output         : none
* Return         : None
******************************************************************************/
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
}


/******************************************************************************
* Function Name  : readFileBin
* Description    : read bin file from SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
*                  - buff: pointer where to store read data 
*                  - len: number fo byte to read 
* Output         : none
* Return         : None
******************************************************************************/
void readFileBin(fs::FS &fs, const char * path, uint8_t * buff, size_t len){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        file.read((uint8_t *)buff, (size_t) len);
    }
}


/******************************************************************************
* Function Name  : writeFileBin
* Description    : write bin file to SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
*                  - buff: pointer from to read data 
*                  - len: number fo byte to write 
* Output         : none
* Return         : None
******************************************************************************/
void writeFileBin(fs::FS &fs, const char * path, uint8_t * message, size_t len){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.write(message, (size_t)len)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
}


/******************************************************************************
* Function Name  : appendFile
* Description    : append ascii file to SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file 
*                  - message: ascii string to append 
* Output         : none
* Return         : None
******************************************************************************/
void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
}


/******************************************************************************
* Function Name  : renameFile
* Description    : rename file SPIFFS
* Input          : - fs: pointer to fs
*                  - path1: complete path of the file to rename
*                  - path2: complete path of the file of new name 
* Output         : none
* Return         : None
******************************************************************************/
void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}


/******************************************************************************
* Function Name  : deleteFile
* Description    : delete file SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file to delete
* Output         : none
* Return         : None
******************************************************************************/
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}


/******************************************************************************
* Function Name  : testFileIO
* Description    : test file IO SPIFFS
* Input          : - fs: pointer to fs
*                  - path: complete path of the file to be used to test
* Output         : none
* Return         : None
******************************************************************************/
void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}


void DRW_Info() {
  unsigned long previousMillis, currentMillis = 0;
 
  LCD_DisplayOff();

  LCD_Clear(BLACK);

  LCD_DrawLine(30, 20, 290, 20, BLUE);
  LCD_DrawLine(30, 20, 30, 210, BLUE);
  LCD_DrawLine(290, 20, 290, 210, BLUE);
  LCD_DrawLine(30, 210, 290, 210, BLUE);
  
  LCD_Small(35, 35,  "        Mini Luxury CAR", GREEN, BLACK);
  LCD_Small(35, 50,  "          Version 1.0", GREEN, BLACK);
  LCD_Small(35, 80,  "  Developed by: UPG S.r.l.s.", WHITE, BLACK);
  LCD_Small(35, 95,  "      email: info@upg.it", WHITE, BLACK);
  //LCD_Small(37, 125, "   Graphics: Andrea Marotta", WHITE, BLACK);
  //LCD_Small(35, 140, "   Algorithm: Andrea Marotta", WHITE, BLACK);
  //LCD_Small(35, 170, "    Inspired by a project of", WHITE, BLACK);
  //LCD_Small(35, 185, "        ", WHITE, BLACK);
  
  LCD_DisplayOn();

  previousMillis = millis();

  //delay(3000);

  while (!myTouch.dataAvailable()) { // wait for touch
    currentMillis = millis();
    if(currentMillis - previousMillis > 4000L) {
      return;  
    }
  }
}


void DRW_Screen1(char *data1) {
  LCD_DisplayOff();

  LCD_Clear(BLACK);

  TP_ResetButton();

  LCD_DrawBox(9, 5, 144, 40, WHITE, BLACK);
  LCD_DrawKeypad(4, 45);

  LCD_Button(160, 5, 270, 25, WHITE, BLUE, 5, 5, "Store here", 14);
  LCD_Text(160, 35, data1, CYAN, BLACK);

  LCD_Button(280, 5, 319, 25, WHITE, BLUE, 5, 5, "Cal", 15);

  LCD_Small(10, 205, "Mini Luxury Car ", WHITE, BLACK);
  LCD_Small(10, 220, "Version 1.0      ", WHITE, BLACK);

  LCD_DisplayOn();
}


void setup() {
  Serial.begin(115200);
  
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  listDir(SPIFFS, "/", 0);
  
  vspi = new SPIClass(VSPI);
  vspi2 = new SPIClass(VSPI);

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  //vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); 
  vspi->begin(); 
  vspi2->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, TP_CS); 
  
  pinMode(VSPI_SS, OUTPUT);
  pinMode(TP_CS, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RESET, OUTPUT);
  pinMode(ONE_LED, OUTPUT);
  pinMode(TP_IRQ, INPUT); 
  
  LCD_Init_9320(LCD_ALIGNMENT);
  //LCD_Init_9325(LCD_ALIGNMENT);

  myTouch.InitTouch(LCD_ALIGNMENT);
  myTouch.setPrecision(PREC_MEDIUM);
  myTouch.calibrateRead();

  DRW_Info();
}


void loop() {
  int i, p = 0;
  char Enter[10] = "        ";

  float center=0, stp=0;
  char center_s[10] = "0       ";
  
  DRW_Screen1(center_s);
  LCD_Rectangle(160,80,319,239,WHITE);

  while (true) {
    if (myTouch.dataAvailable()) {
      myTouch.read();
//      Serial.print(Screen.x);
//      Serial.print(" /S/ ");
//      Serial.println(Screen.y);
      getDisplayPoint();
//      Serial.print(disp.x);
//      Serial.print(" /D/ ");
//      Serial.println(disp.y);
      if ( (disp.x > 160) && (disp.x < 319) && (disp.y > 80) && (disp.y < 239) )
        LCD_SetPoint(disp.x, disp.y, WHITE);
      if (disp.x == 0 && disp.y == 0) {
        myTouch.calibrateRead();
        DRW_Screen1(center_s);
        p = 0;
        strcpy(Enter, "        ");
      }

      if ((i = TP_Button()) != -1)  {
        switch (i) {
          case 0:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p != 0) { // number cannot start with 0
              if (p < 8) {
                Enter[p] = '0';
                p++;
                LCD_Text(13, 7, Enter, CYAN, BLACK);
              }
            }
            break;
          case 1:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '1';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 2:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '2';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 3:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '3';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 4:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '4';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 5:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '5';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 6:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '6';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 7:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '7';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 8:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '8';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 9:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p < 8) {
              Enter[p] = '9';
              p++;
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 10:
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p != 0) { // number cannot start with 000
              if (p < 6) {
                Enter[p] = '0';
                p++;
                Enter[p] = '0';
                p++;

                Enter[p] = '0';
                p++;
                LCD_Text(13, 7, Enter, CYAN, BLACK);
              }
            }
            break;
          case 11: // 00
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p != 0) { // number cannot start with 00
              if (p < 7) {
                Enter[p] = '0';
                p++;
                Enter[p] = '0';
                p++;
                LCD_Text(13, 7, Enter, CYAN, BLACK);
              }
            }
            break;
          case 12: // backspace
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (p >= 1) {
              p--;
              Enter[p] = ' ';
              LCD_Text(13, 7, Enter, CYAN, BLACK);
            }
            break;
          case 13: // clr
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            strcpy(Enter, "        ");
            LCD_Text(13, 7, Enter, CYAN, BLACK);
            p = 0;
            break;
          case 14: // center
            LCD_Small(10, 205, "                 ", WHITE, BLACK);
            LCD_Small(10, 220, "                 ", WHITE, BLACK);
            if (atof(Enter) != 0) {
              if (atof(Enter) >= 0 && atof(Enter) <= 99999999) { 
                center = atof(Enter);
                strcpy(center_s, Enter);
                LCD_Text(160, 35, center_s, CYAN, BLACK);
                LCD_Small(10, 205, "                 ", WHITE, BLACK);
                LCD_Small(10, 220, "                 ", WHITE, BLACK);
              } else {
                LCD_Small(10, 205, "                 ", WHITE, BLACK);
                LCD_Small(10, 220, "                 ", WHITE, BLACK);
                LCD_Small(10, 205, "Incorrect value  ", WHITE, BLACK);
              }
            } else {
              LCD_Small(10, 205, "                 ", WHITE, BLACK);
              LCD_Small(10, 220, "                 ", WHITE, BLACK);
              LCD_Small(10, 205, "Enter area empty ", WHITE, BLACK);
            }
            break;
          case 15: // cal
            if (orien == VERTICAL) deleteFile(SPIFFS, "/vertical.cal");
            else deleteFile(SPIFFS, "/horizontal.cal");
            myTouch.calibrateRead();
            DRW_Screen1(center_s);
            LCD_Rectangle(160,80,319,239,WHITE);
            p = 0;
            strcpy(Enter, "        ");
            break;
          default:
           // Code
            break;
        }
      } else {
        if ( (disp.x > 10) && (disp.x < 149) && (disp.y > 205) && (disp.y < 239) ) {
          DRW_Screen1(center_s);
          LCD_Rectangle(160,80,319,239,WHITE);
          p = 0;
          strcpy(Enter, "        ");
        }
      }
    }
    //delay(5);
  }
}
