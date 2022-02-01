import json
from pathlib import Path

from PIL import Image as PILImage
from django.conf import settings
from django.shortcuts import render, redirect, get_object_or_404
from django.views import View

from patron.models import Image
from patron.patron_main import test_func


class IndexView(View):
    template_name = 'patron/index.html'

    def get(self, request, *args, **kwargs):
        images = {'images': Image.objects.all()}
        return render(request, self.template_name, context=images)

    # noinspection PyMethodMayBeStatic
    def post(self, request):
        image = request.FILES.get('image-file')
        image_name = str(image.name).replace(' ', '_')
        uploaded_image = Image(name=image_name, image=image)
        uploaded_image.save()
        return redirect('image_view', image_id=uploaded_image.pk)


class ImageView(View):
    template_name = 'patron/image_view.html'

    @staticmethod
    def get_context_from_image(image):
        paper_size = json.loads(image.paper_size)
        num_img = json.loads(image.num_img)
        labels = json.loads(json.loads(json.dumps(image.labels, ensure_ascii=False)))
        rend_size = json.loads(image.rend_size)
        return {
            'image': image.name,
            'imgId': image.pk,
            'paperSize': paper_size,
            'numImg': num_img,
            'labels': labels,
            'rendSize': rend_size
        }

    def get(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        context = self.get_context_from_image(image)
        return render(request, self.template_name, context=context)

    def post(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        json_data = json.loads(request.POST.get('image-data', "{}"))
        image.labels = str(json_data['labels']).replace("'", '"')
        image.paper_size = str(json_data['paper']).replace("'", '"')
        image.num_img = str(json_data['copies']).replace("'", '"')
        image.rend_size = str(json_data['imgSize']).replace("'", '"')
        image.save()

        # context = self._get_context_from_image(image)
        return redirect('pdf_download', image_id)
        # return render(request, self.template_name, context=context)


class PdfDownloadView(View):
    template_name = 'patron/pdf_download.html'

    def get(self, request, image_id):
        test_func(image_id)
        return render(request, self.template_name)
        db_image = get_object_or_404(Image, pk=image_id)
        img_path = Path(db_image.image.file.name)
        img_path = '{}/{}'.format(img_path.parent, db_image.name)
        image = PILImage.open(img_path)
        print('original: width: {}, height: {}'.format(image.width, image.height))
        context = ImageView.get_context_from_image(db_image)
        rend_size = context['rendSize']
        print('rendered: width: {}, height: {}'.format(rend_size['imgWidth'], rend_size['imgHeight']))
        for item in context['labels']:
            print(item)
        return render(request, self.template_name)

    def post(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        return render(request, self.template_name)
