import json
import uno
from com.sun.star.beans import PropertyValue
import serial
from serial.tools import list_ports
import sys
import traceback
from datetime import datetime
from com.sun.star.util import DateTime as UnoDateTime

def connect_to_libreoffice():
    """Подключается к LibreOffice и создает новый документ"""
    try:
        local_context = uno.getComponentContext()
        resolver = local_context.ServiceManager.createInstanceWithContext(
            "com.sun.star.bridge.UnoUrlResolver", local_context)

        context = resolver.resolve("uno:socket,host=localhost,port=2002;urp;StarOffice.ComponentContext")
        desktop = context.ServiceManager.createInstanceWithContext("com.sun.star.frame.Desktop", context)

        # Создаем абсолютно новый документ
        # doc = desktop.loadComponentFromURL("private:factory/scalc", "_blank", 0, ())
        doc = desktop.getCurrentComponent()

        # Проверяем инициализацию
        if not hasattr(doc, "getSheets"):
            raise RuntimeError("Созданный документ не является таблицей")

        # Добавляем первый лист, если документ пустой
        if doc.getSheets().getCount() == 0:
            doc.getSheets().insertNewByName("Default", 0)

        return doc

    except Exception as e:
        print(f"Ошибка подключения: {traceback.format_exc()}")
        sys.exit(1)


def dt_to_office_format(dt_object):
    # Конвертируем в формат LibreOffice (количество дней с 30.12.1899)
    # Дата
    lo_date = dt_object.date().toordinal() - datetime(1899, 12, 30).toordinal()
    # Время (доля от дня)
    lo_time = (dt_object.hour * 3600 + dt_object.minute * 60 + dt_object.second) / 86400.0
    # Общее значение
    return lo_date + lo_time

if __name__ == "__main__":
    document = connect_to_libreoffice()
    
    sheet = document.getSheets().getByIndex(0)
    sdt = "2025-05-19 09:31:38"
    cell = sheet.getCellByPosition(0, 20)

    locale = document.CharLocale
    number_formats = document.getNumberFormats()
    # Ищем существующий формат
    format_string = "YYYY-MM-DD HH:MM:SS"
    format_id = number_formats.queryKey(format_string, locale, True)

    print(f"format : {format_id}")
    color = cell.CellBackColor
    print(f"color : {color}")

    try:
        cell.Value = dt_to_office_format(datetime.fromisoformat(sdt))
        cell.NumberFormat = 10073
    except Exception as e:
        print(f"Ошибка: {traceback.format_exc()}")            
