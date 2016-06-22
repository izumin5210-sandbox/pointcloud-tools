from primesense import openni2
from pctools.ni_device import NiDevice

import click


@click.command()
@click.argument("onifile",
                type=click.Path(exists=True))
@click.option('--yamlfile',
              default='kinect2.yaml',
              type=click.Path(exists=True),
              help='OpenCV configuration file.')
def cmd(onifile, yamlfile):
    openni2.initialize()

    device = NiDevice(onifile, yamlfile)
    device.initialize()
    device.update()

    openni2.unload()


def main():
    cmd()


if __name__ == '__main__':
    main()

