from django.shortcuts import render, redirect
from django.views import View

from patron.models import Image


class IndexView(View):
    template_name = 'patron/index.html'

    def get(self, request, *args, **kwargs):
        images = {'images': Image.objects.all()}
        return render(request, self.template_name, context=images)

    def post(self, request):
        image = request.FILES.get('image-file')
        image_name = str(image.name).replace(' ', '_')
        uploaded_image = Image(name=image_name, image=image)
        uploaded_image.save()
        return redirect('image_view', image_id=uploaded_image.pk)


class ImageView(View):
    template_name = 'patron/image_view.html'

    def get(self, request, image_id):
        return render(request, self.template_name, context={'image': image_id})
