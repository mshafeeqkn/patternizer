from django.http import HttpResponse
from django.shortcuts import render
from django.views import View

from patron.models import Image


class IndexView(View):
    template_name = 'patron/index.html'

    def get(self, request, *args, **kwargs):
        return render(request, self.template_name)

    def post(self,request):
        image = request.FILES.get('image-file')
        uploaded_image = Image(name='sample.jpg', image=image)
        uploaded_image.save()
        return render(request, self.template_name)