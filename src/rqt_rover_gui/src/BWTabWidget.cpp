#include "BWTabWidget.h"

#include <QPalette>
#include <QTabBar>

BWTabWidget::BWTabWidget(QWidget *p) : QTabWidget(p)
{
  QPalette pal = tabBar()->palette();
  pal.setColor(QPalette::WindowText, QColor(255,255,255));


  pal.setColor(QPalette::Window, QColor(0,0,0));
  pal.setColor(QPalette::Button, QColor(0,0,0));
  pal.setColor(QPalette::ButtonText, QColor(0,0,0));
  pal.setColor(QPalette::Base, QColor(0, 0, 0));
  pal.setColor(QPalette::AlternateBase, QColor(0, 0, 0));
  pal.setColor(QPalette::Text, Qt::black);
  tabBar()->setPalette(pal);
}
