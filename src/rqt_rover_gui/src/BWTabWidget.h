/*!
 * \brief  This subclass just provides a way to change the color of the
 *         QTabWidget in the GUI. Changing colors isn't possible just using
 *         the UI editor.
 * \author Matthew Fricke
 * \date   November 11th 2015
 * \todo   Code works properly.
 * \class  BQTabWidget
 */

#ifndef BWTABWIDGET_H
#define BWTABWIDGET_H

#include <QTabWidget>

class BWTabWidget : public QTabWidget {
  public:
    BWTabWidget(QWidget *p=0);
};

#endif // BWTABWIDGET_H
