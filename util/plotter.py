#!/usr/bin/env python3

from sys import argv

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mpld3
import pandas as pd


KML_HEADER = '<?xml version="1.0" encoding="UTF-8"?>\n\
<kml xmlns="http://earth.google.com/kml/2.0">\n\
<Document>\n'

KML_FOOTER = '</Document>\n</kml>'

LINE_HEADER = '\
<Placemark>\n\
<LineString>\n\
<coordinates>\n'

LINE_FOOTER = '\
</coordinates>\n\
</LineString>\n\
<Style>\n\
<LineStyle>\n\
<color>#ff0000ff</color>\n\
<width>5</width>\n\
</LineStyle>\n\
</Style>\n\
</Placemark>\n'

POINT_HEADER='<Placemark>\n<Point>\n<coordinates>\n'

POINT_FOOTER='</coordinates>\n</Point>\n</Placemark>\n'


def plot(file_name, x_axis, y_axis):

    fig0 = plt.figure(0, figsize=(10, 8))
    plt.suptitle(file_name)

    data = pd.read_csv(file_name)

    ax = plt.subplot(1, 1, 1)
    ax.set_title("plot")
    ax.set_aspect('equal', 'datalim')

    ax.plot(data[x_axis], data[y_axis], color='b',
            label="data", marker='o', markersize=5, linewidth=2)

    ax.set_xlabel(x_axis)
    ax.set_ylabel(y_axis)
    ax.grid(which="both")
    # ax.legend(loc='best', numpoints=1)
    html_name = file_name + '_plot.html'

    mpld3.save_html(fig0, html_name)
    plt.close()

    print("Save:" + html_name)
    return html_name


def writeKML(file_name, long_axis, lat_axis):

    data = pd.read_csv(file_name)

    kml = KML_HEADER

    kml += LINE_HEADER
    for lon, lat in zip(data[long_axis], data[lat_axis]):
        kml += str(lon) + ',' + str(lat) +',0.0\n'
    kml += LINE_FOOTER

    # for lon, lat in zip(data[long_axis], data[lat_axis]):
    #     kml += POINT_HEADER
    #     kml += str(lon) + ',' + str(lat) +',0.0\n'
    #     kml += POINT_FOOTER

    kml += KML_FOOTER

    kml_name = file_name + '.kml'

    with open(kml_name, mode='w') as f:
        f.write(kml)
        f.close()

    print("Save:" + kml_name)
    return kml_name


if __name__ == '__main__':

    data_csv = argv[1]

    x_axis = 'msg.pose.position.x'
    y_axis = 'msg.pose.position.y'

    plot(data_csv, x_axis, y_axis)
