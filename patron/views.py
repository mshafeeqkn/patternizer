import json

from django.http import HttpResponse
from django.shortcuts import render, redirect, get_object_or_404
from django.views import View

from patron.models import Image


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
    def _get_context_from_image(image):
        paper_size = json.loads(image.paper_size)
        num_img = json.loads(image.num_img)
        labels = json.dumps(image.labels, ensure_ascii=False)
        return {
            'image': image.name,
            'imgId': image.pk,
            'paperSize': paper_size,
            'numImg': num_img,
            'labels': labels
        }

    def get(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        context = self._get_context_from_image(image)
        return render(request, self.template_name, context=context)

    def post(self, request, image_id):
        image = get_object_or_404(Image, pk=image_id)
        json_data = json.loads(request.POST.get('image-data', "{}"))
        image.labels = str(json_data['labels']).replace("'", '"')
        image.paper_size = str(json_data['paper']).replace("'", '"')
        image.num_img = str(json_data['copies']).replace("'", '"')
        image.save()

        context = self._get_context_from_image(image)
        return render(request, self.template_name, context=context)


class ImageDataView(View):
    def get(self, request, image_id):
        return HttpResponse("{'key': 'value'}")