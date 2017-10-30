/****************************************************************************
** Meta object code from reading C++ file 'my_plugin.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/ucl_drone_gui/include/ucl_drone_gui/my_plugin.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'my_plugin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ucl_drone_gui__MyPlugin[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   25,   24,   24, 0x0a,
      51,   24,   24,   24, 0x0a,
      71,   24,   24,   24, 0x0a,
      86,   24,   24,   24, 0x0a,
     104,   24,   24,   24, 0x0a,
     125,   24,   24,   24, 0x0a,
     145,   24,   24,   24, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ucl_drone_gui__MyPlugin[] = {
    "ucl_drone_gui::MyPlugin\0\0str\0"
    "drone_select(QString)\0emergency_publish()\0"
    "land_publish()\0takeoff_publish()\0"
    "reset_pose_publish()\0add_drone_to_list()\0"
    "remove_drone_from_list()\0"
};

void ucl_drone_gui::MyPlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyPlugin *_t = static_cast<MyPlugin *>(_o);
        switch (_id) {
        case 0: _t->drone_select((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->emergency_publish(); break;
        case 2: _t->land_publish(); break;
        case 3: _t->takeoff_publish(); break;
        case 4: _t->reset_pose_publish(); break;
        case 5: _t->add_drone_to_list(); break;
        case 6: _t->remove_drone_from_list(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ucl_drone_gui::MyPlugin::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ucl_drone_gui::MyPlugin::staticMetaObject = {
    { &rqt_gui_cpp::Plugin::staticMetaObject, qt_meta_stringdata_ucl_drone_gui__MyPlugin,
      qt_meta_data_ucl_drone_gui__MyPlugin, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ucl_drone_gui::MyPlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ucl_drone_gui::MyPlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ucl_drone_gui::MyPlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ucl_drone_gui__MyPlugin))
        return static_cast<void*>(const_cast< MyPlugin*>(this));
    typedef rqt_gui_cpp::Plugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int ucl_drone_gui::MyPlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rqt_gui_cpp::Plugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
