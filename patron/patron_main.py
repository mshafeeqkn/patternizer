from pathlib import Path

from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4, A2
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

from patron.models import Image, LFont
from patternizer import settings


class GeneratePatron:
    def __init__(self, file_name, image_id, num_copies=(1, 1), paper_size=A4, padding=0):
        self.canvas = canvas.Canvas(file_name, paper_size)
        self.num_row, self.num_col = num_copies
        self.paper_size = paper_size
        db_image = Image.objects.get(pk=image_id)
        self.img_width = db_image.image.width
        self.img_height = db_image.image.height
        img_path = Path(db_image.image.file.name)
        self.bg_image = '{}/{}'.format(img_path.parent, db_image.name)
        self.register_fonts()
        self.padding = padding
        self.img_render_height = (self.paper_size[1] - 2 * self.num_row * self.padding) / self.num_row
        self.img_render_width = self.img_width * self.img_render_height / self.img_height

    def __del__(self):
        self.canvas.save()
        print('Canvas saved')

    @staticmethod
    def register_fonts():
        font_list = LFont.objects.all()
        for font in font_list:
            font_key = Path(font.font.name).stem
            if font_key not in pdfmetrics.getRegisteredFontNames():
                pdfmetrics.registerFont(TTFont(font_key, '{}/{}'.format(settings.MEDIA_ROOT, font.font.name)))
                print('Loading image: {}'.format(font.font.name))
                print(pdfmetrics.getRegisteredFontNames())

    def set_background_image(self, row, col):
        x = col * (self.img_render_width + 2 * self.padding) + self.padding
        y = row * (self.paper_size[1] / self.num_row) + self.padding

        self.canvas.drawImage(self.bg_image, x, y, self.img_render_width, preserveAspectRatio=True, anchor='sw')
        self.canvas.setStrokeColor(HexColor(0xD3D3D3))
        self.canvas.setLineWidth(0.5)

        # Horizontal line
        self.canvas.line(col * (self.paper_size[0] / self.num_col),
                         row * (self.paper_size[1] / self.num_row),
                         (col + 1) * (self.paper_size[0] / self.num_col),
                         row * (self.paper_size[1] / self.num_row))

        # Vertical line
        self.canvas.line((col + 1) * (self.img_render_width + 2 * self.padding),
                         (row + 1) * (self.paper_size[1] / self.num_row),
                         (col + 1) * (self.img_render_width + 2 * self.padding),
                         row * (self.paper_size[1] / self.num_row))

        self.canvas.setStrokeColor(HexColor(0))


def test_func(image_id):
    dir_name = '{}/{}'.format(settings.MEDIA_ROOT, 'pdf')
    file_name = '{}/generated_output.pdf'.format(dir_name)
    Path(dir_name).mkdir(parents=True, exist_ok=True)
    gen = GeneratePatron(file_name=file_name, image_id=image_id, paper_size=A4, num_copies=(3, 1), padding=5)
    for i in range(1):
        for j in range(3):
            gen.set_background_image(row=j, col=i)
