import json
from json import JSONDecodeError
from pathlib import Path

import xlrd
from PIL import Image as PILImage
from django.shortcuts import render, redirect, get_object_or_404
from django.views import View

from patron.models import Image
from patron.patron_main import generate_patron
from patternizer.settings import MEDIA_ROOT


class IndexView(View):
    template_name = 'patron/index.html'

    def get(self, request, *args, **kwargs):
        images = {'images': Image.objects.all()}
        return render(request, self.template_name, context=images)

    # noinspection PyMethodMayBeStatic
    def post(self, request):
        image = request.FILES.get('image-file')
        data = request.FILES.get('data-file')
        image_name = str(image.name).replace(' ', '_')
        uploaded_image = Image(name=image_name, image=image, data_file=data)
        uploaded_image.save()
        return redirect('image_view', image_id=uploaded_image.pk)


class ImageView(View):
    template_name = 'patron/image_view.html'
    labels = []
    img_data = []

    def load_data_from_excel(self, image):
        self.labels.clear()
        self.img_data.clear()

        data_file = image.data_file
        wb = xlrd.open_workbook('{}/{}'.format(MEDIA_ROOT, data_file.name))
        sheet = wb.sheet_by_index(0)
        self.labels += sheet.row_values(0)

        for i in range(1, sheet.nrows):
            self.img_data.append(list(sheet.row_values(i)))
            self.img_data[-1][-1] = str(self.img_data[-1][-1])[:-2]

    def get_context_from_image(self, image):
        self.load_data_from_excel(image)

        return {
            'image': image.name,
            'imgId': image.pk,
            'labels': self.labels,
            'imgData': self.img_data
        }

    def get(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        context = self.get_context_from_image(image)
        return render(request, self.template_name, context=context)

    def _generate_pdf(self, image, json_data):
        num_copies = tuple([int(d) for d in json_data['copies'].values()])
        paper_size = [int(d) for d in json_data['paper'].values()]
        generate_patron(image_id=image.pk, num_copies=num_copies, paper_size=paper_size, padding=2)

    def post(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        json_data = json.loads(request.POST.get('image-data', "{}"))
        self._generate_pdf(image, json_data)
        context = self.get_context_from_image(image)
        return render(request, self.template_name, context=context)
