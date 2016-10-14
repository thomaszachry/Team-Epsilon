#include "window.h"

#include <QApplication>

Window::Window(QWidget *parent) :
 QWidget(parent)
 {
  // Set size of the window
  setFixedSize(400, 400);

  // Create and position the button
  m_button = new QPushButton("Exit", this);
  m_button->setGeometry(315, 350, 80, 30);

  // NEW : Do the connection
  connect(m_button, SIGNAL (clicked()), QApplication::instance(), SLOT (quit()));
 }
