#include "pclviewer.h"

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLViewer w;

  //set position of cmd window (for debug)
  //cmd window will not show in release mode
  HWND consoleWindow = GetConsoleWindow();
  SetWindowPos(consoleWindow, 0, 600, 0, 600, 380, SWP_NOZORDER);
  //SetWindowPos(HWND hWnd, HWND hWndInsertAfter, int  X, int  Y, int  cx, int  cy, UINT uFlags)


  //set position of cv window (show input)
  cv::namedWindow("kinect");
  cvMoveWindow("kinect", 0, 0);

  //cv::namedWindow("projection");
  //cvMoveWindow("projection", 0, 0);

  //set position of ui window(get user input, show output)
  w.show ();
  w.move(0, 340);

  return a.exec ();
}
