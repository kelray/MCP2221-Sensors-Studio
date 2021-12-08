/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../SensorsStudioV2/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[17];
    char stringdata0[257];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 19), // "MMA8452_updateChart"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 23), // "MMA8452_DropBoxCallback"
QT_MOC_LITERAL(4, 56, 5), // "index"
QT_MOC_LITERAL(5, 62, 21), // "MMA8452_ChkBxCallback"
QT_MOC_LITERAL(6, 84, 12), // "ConnectToCOM"
QT_MOC_LITERAL(7, 97, 10), // "GetGPSinfo"
QT_MOC_LITERAL(8, 108, 21), // "MPU9250_ChkBxCallback"
QT_MOC_LITERAL(9, 130, 19), // "MPU9250_updateChart"
QT_MOC_LITERAL(10, 150, 17), // "MPU9250_GyroScale"
QT_MOC_LITERAL(11, 168, 16), // "MPU9250_AccScale"
QT_MOC_LITERAL(12, 185, 22), // "AnalogIn_ChkBxCallback"
QT_MOC_LITERAL(13, 208, 20), // "AnalogIn_updateChart"
QT_MOC_LITERAL(14, 229, 7), // "ExitApp"
QT_MOC_LITERAL(15, 237, 8), // "AboutApp"
QT_MOC_LITERAL(16, 246, 10) // "ActiveTabs"

    },
    "MainWindow\0MMA8452_updateChart\0\0"
    "MMA8452_DropBoxCallback\0index\0"
    "MMA8452_ChkBxCallback\0ConnectToCOM\0"
    "GetGPSinfo\0MPU9250_ChkBxCallback\0"
    "MPU9250_updateChart\0MPU9250_GyroScale\0"
    "MPU9250_AccScale\0AnalogIn_ChkBxCallback\0"
    "AnalogIn_updateChart\0ExitApp\0AboutApp\0"
    "ActiveTabs"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   84,    2, 0x0a /* Public */,
       3,    1,   85,    2, 0x0a /* Public */,
       5,    1,   88,    2, 0x0a /* Public */,
       6,    0,   91,    2, 0x0a /* Public */,
       7,    0,   92,    2, 0x0a /* Public */,
       8,    1,   93,    2, 0x0a /* Public */,
       9,    0,   96,    2, 0x0a /* Public */,
      10,    1,   97,    2, 0x0a /* Public */,
      11,    1,  100,    2, 0x0a /* Public */,
      12,    1,  103,    2, 0x0a /* Public */,
      13,    0,  106,    2, 0x0a /* Public */,
      14,    0,  107,    2, 0x0a /* Public */,
      15,    0,  108,    2, 0x0a /* Public */,
      16,    1,  109,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->MMA8452_updateChart(); break;
        case 1: _t->MMA8452_DropBoxCallback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->MMA8452_ChkBxCallback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->ConnectToCOM(); break;
        case 4: _t->GetGPSinfo(); break;
        case 5: _t->MPU9250_ChkBxCallback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->MPU9250_updateChart(); break;
        case 7: _t->MPU9250_GyroScale((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->MPU9250_AccScale((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->AnalogIn_ChkBxCallback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->AnalogIn_updateChart(); break;
        case 11: _t->ExitApp(); break;
        case 12: _t->AboutApp(); break;
        case 13: _t->ActiveTabs((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
