from django.urls import path

from patron import views

urlpatterns = [
    path('', views.IndexView.as_view(), name='patron'),
    path('<int:image_id>/', views.ImageView.as_view(), name='image_view'),
    path('image-data/<int:image_id>/', views.ImageDataView.as_view(), name='image_data')
]
